# Copyright 2026 Zheng Qu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
Test-only Protocol 2 bus; no physical serial device is ever opened.

The register subset/layouts follow ROBOTIS 1.5.1 XL430 and XL330 fw52 models.
Packet framing follows DynamixelSDK 4.0.3 protocol2_packet_handler and the
GroupFast{Bulk,Sync}Read parsers. No motor dynamics, USB latency, thermal or
power behavior is modelled. Instant goal tracking is deliberately idealized.
"""

import os
import pty
import select
import threading
import time
import tty


HEADER = b'\xff\xff\xfd\x00'


def crc16(data):
    """Calculate the Protocol 2 CRC (polynomial 0x8005, initial zero)."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x8005 if crc & 0x8000 else 0)) & 0xffff
    return crc


def word(value):
    """Encode an unsigned little-endian word."""
    return value.to_bytes(2, 'little')


def number(data):
    """Decode an unsigned little-endian register."""
    return int.from_bytes(data, 'little')


def packet(servo_id, body):
    """Frame and stuff a standard packet, including its CRC."""
    body = body.replace(b'\xff\xff\xfd', b'\xff\xff\xfd\xfd')
    result = HEADER + bytes([servo_id]) + word(len(body) + 2) + body
    return result + word(crc16(result))


class DynamixelBus:
    """Own a pty bus and a worker, with recorded packets and register writes."""

    def __init__(self, *, fast_read=True, torque_on=()):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)
        assert self.port.startswith('/dev/pts/')
        self.fast_read = fast_read
        self.registers = {i: bytearray(1024) for i in range(1, 7)}
        for i in self.registers:
            self.set_register(i, 0, 2, 1060 if i <= 2 else 1200)
            self.set_register(i, 6, 1, 48 if i <= 2 else 52)
            self.set_register(i, 7, 1, i)
            self.set_register(i, 11, 1, 3)
            self.set_register(i, 64, 1, int(i in torque_on))
            self.set_register(i, 132, 4, 2048 + 10 * (i - 1))
            self.set_register(i, 128, 4, -i)
            self.set_register(i, 126, 2, -i)
        self.instructions = []
        self.writes = []
        self.reads = []
        self.drop_cycles = 0
        self.drop_forever = False
        self.silent_ids = set()
        self.silent_cycles = {}
        self.corrupt_cycles = 0
        self.error_status_ids = set()
        self.hardware_errors_at_group_read = {}
        self.short_fast_cycles = 0
        self.fault_events = []
        self.valid_group_replies = []
        self.reboots = []
        self.ignored_protocol1_pings = []
        self.error = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, name='dxl_sim', daemon=True)
        self.thread.start()

    def set_register(self, servo_id, address, size, value):
        """Inject raw state (negative values use two's complement)."""
        value &= (1 << (8 * size)) - 1
        self.registers[servo_id][address:address + size] = value.to_bytes(size, 'little')

    def _address(self, servo_id, address):
        layouts = [(224, 168, 28), (634, 578, 28)] if servo_id <= 2 else [
            (208, 168, 6), (214, 180, 14)]
        for data, indirect, size in layouts:
            if data <= address < data + size:
                offset = indirect + 2 * (address - data)
                return number(self.registers[servo_id][offset:offset + 2])
        return address

    def _read(self, servo_id, address, size):
        data = bytes(self.registers[servo_id][self._address(servo_id, address + j)]
                     for j in range(size))
        self.reads.append((time.monotonic(), servo_id, address, data))
        return data

    def _write(self, servo_id, address, data):
        memory = self.registers[servo_id]
        addresses = [self._address(servo_id, address + j) for j in range(len(data))]
        torque = memory[64]
        error = 7 if torque and any(a < 64 for a in addresses) else 0
        self.writes.append({
            'time': time.monotonic(), 'id': servo_id, 'address': address,
            'resolved': addresses, 'data': list(data), 'torque': torque, 'error': error})
        if error:
            return error
        for target, value in zip(addresses, data):
            memory[target] = value
        if memory[64] and (64 in addresses or any(116 <= a < 120 for a in addresses)):
            memory[132:136] = memory[116:120]
        return 0

    def _alert(self, servo_id):
        return 0x80 if self.registers[servo_id][70] else 0

    def _reply(self, servo_id, data=b'', error=0):
        return packet(servo_id, bytes([0x55, error | self._alert(servo_id)]) + data)

    def _group_read(self, requests, fast):
        for servo_id, value in self.hardware_errors_at_group_read.items():
            self.set_register(servo_id, 70, 1, value)
        if self.drop_forever or self.drop_cycles:
            self.fault_events.append((time.monotonic(), 'drop', fast))
            self.drop_cycles = max(0, self.drop_cycles - 1)
            return b''
        silent = self.silent_ids | {i for i, count in self.silent_cycles.items() if count}
        self.silent_cycles = {i: max(0, count - 1) for i, count in self.silent_cycles.items()}
        if any(i in silent for i, _, _ in requests):
            self.fault_events.append((time.monotonic(), 'silent', fast))
        if fast and (not self.fast_read or any(i in silent for i, _, _ in requests)):
            return b''
        faulty = bool(silent or self.error_status_ids or self.corrupt_cycles)
        if fast and self.short_fast_cycles:
            self.short_fast_cycles -= 1
            self.fault_events.append((time.monotonic(), 'short_fast', fast))
            requests = requests[:3]
            faulty = True
        if fast:
            # Fast replies are one unstuffed broadcast status. Each segment has
            # error, ID, data, cumulative CRC, including the final packet CRC.
            length = 1 + sum(size + 4 for _, _, size in requests)
            result = HEADER + b'\xfe' + word(length) + b'\x55'
            for i, addr, size in requests:
                result += bytes([self._alert(i), i]) + self._read(i, addr, size)
                result += word(crc16(result))
        else:
            replies = []
            for i, addr, size in requests:
                if i in silent:
                    continue
                if i in self.error_status_ids:
                    self.fault_events.append((time.monotonic(), 'error_status', fast))
                    replies.append(self._reply(i, error=4))
                else:
                    replies.append(self._reply(i, self._read(i, addr, size)))
            result = b''.join(replies)
        if self.corrupt_cycles:
            self.fault_events.append((time.monotonic(), 'crc', fast))
            self.corrupt_cycles -= 1
            result = result[:-1] + bytes([result[-1] ^ 1])
        if not faulty:
            self.valid_group_replies.append(time.monotonic())
        return result

    def _dispatch(self, servo_id, instruction, params):
        self.instructions.append((time.monotonic(), servo_id, instruction, list(params)))
        if servo_id != 254 and servo_id not in self.registers:
            return b''
        if instruction == 1:
            memory = self.registers[servo_id]
            return self._reply(servo_id, memory[:2] + memory[6:7])
        if instruction == 2:
            if servo_id in self.silent_ids or self.drop_forever:
                return b''
            return self._reply(servo_id, self._read(
                servo_id, number(params[:2]), number(params[2:4])))
        if instruction == 3:
            error = self._write(servo_id, number(params[:2]), params[2:])
            return self._reply(servo_id, error=error)
        if instruction == 8:
            self.reboots.append((time.monotonic(), servo_id, self.registers[servo_id][64]))
            self.set_register(servo_id, 64, 1, 0)
            self.set_register(servo_id, 70, 1, 0)
            return self._reply(servo_id)
        if instruction in (0x82, 0x8a):
            requests = [(i, number(params[:2]), number(params[2:4])) for i in params[4:]]
            return self._group_read(requests, instruction == 0x8a)
        if instruction in (0x92, 0x9a):
            requests = [(params[j], number(params[j + 1:j + 3]),
                         number(params[j + 3:j + 5])) for j in range(0, len(params), 5)]
            return self._group_read(requests, instruction == 0x9a)
        if instruction == 0x83:
            address, size = number(params[:2]), number(params[2:4])
            for j in range(4, len(params), size + 1):
                self._write(params[j], address, params[j + 1:j + 1 + size])
            return b''
        if instruction == 0x93:
            j = 0
            while j < len(params):
                i, address, size = params[j], number(params[j + 1:j + 3]), number(
                    params[j + 3:j + 5])
                self._write(i, address, params[j + 5:j + 5 + size])
                j += 5 + size
            return b''
        raise AssertionError(f'Unsupported instruction: {instruction:#x}')

    def _run(self):
        buffer = b''
        try:
            while not self.stopping.is_set():
                if not select.select([self.master], [], [], 0.02)[0]:
                    continue
                buffer += os.read(self.master, 65536)
                while len(buffer) >= 6:
                    # Workbench probes Protocol 1 first. A Protocol 2 servo
                    # ignores that ping; no Protocol 1 registers are emulated.
                    if buffer.startswith(b'\xff\xff') and buffer[2] != 0xfd:
                        length = buffer[3] + 4
                        if len(buffer) < length:
                            break
                        frame, buffer = buffer[:length], buffer[length:]
                        assert frame[3:5] == b'\x02\x01', frame.hex()
                        assert frame[-1] == (~sum(frame[2:-1]) & 0xff)
                        self.ignored_protocol1_pings.append((time.monotonic(), frame[2]))
                        continue
                    if len(buffer) < 7:
                        break
                    assert buffer.startswith(HEADER), buffer.hex()
                    length = 7 + number(buffer[5:7])
                    if len(buffer) < length:
                        break
                    frame, buffer = buffer[:length], buffer[length:]
                    assert crc16(frame[:-2]) == number(frame[-2:])
                    body = frame[7:-2].replace(b'\xff\xff\xfd\xfd', b'\xff\xff\xfd')
                    reply = self._dispatch(frame[4], body[0], body[1:])
                    if reply:
                        os.write(self.master, reply)
        except Exception as exc:
            self.error = exc

    def close(self):
        """Stop the worker and release only this simulator's pty descriptors."""
        self.stopping.set()
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)
        assert not self.thread.is_alive()
        assert self.error is None, repr(self.error)
