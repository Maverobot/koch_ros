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

"""Protocol/register checks independent of the driver's integration assertions."""

from dxl_sim import crc16, DynamixelBus, HEADER, number, packet, word
import pytest


def test_protocol2_crc_and_stuffing():
    """Match the Protocol 2 ID1 ping example and escape payload header bytes."""
    assert packet(1, b'\x01').hex() == 'fffffd0001030001194e'
    frame = packet(1, b'\x55\x00\xff\xff\xfd\xfd')
    assert frame[7:-2] == b'\x55\x00\xff\xff\xfd\xfd\xfd'
    assert len(frame) == 7 + number(frame[5:7])
    assert crc16(frame[:-2]) == number(frame[-2:])


@pytest.mark.parametrize('servo_id,indirect,data', [(1, 578, 634), (3, 180, 214)])
def test_indirect_register_mapping_and_eeprom_access(servo_id, indirect, data):
    """Both real firmware layouts map signed bytes and reject torque-on EEPROM."""
    bus = DynamixelBus()
    try:
        for offset in range(4):
            assert bus._write(servo_id, indirect + 2 * offset, word(132 + offset)) == 0
        bus.set_register(servo_id, 132, 4, -123)
        assert int.from_bytes(bus._read(servo_id, data, 4), 'little', signed=True) == -123
        bus.set_register(servo_id, 64, 1, 1)
        assert bus._write(servo_id, 11, b'\x04') == 7
        assert bus.registers[servo_id][11] == 3
        bus.set_register(servo_id, 64, 1, 0)
        assert bus._write(servo_id, 11, b'\x03') == 0
    finally:
        bus.close()


def test_fast_reply_is_unstuffed_and_has_final_crc():
    """SDK fast parsers consume error/ID/data/CRC segments, without unstuffing."""
    bus = DynamixelBus()
    try:
        bus.set_register(1, 132, 4, 0xfdffff)
        response = bus._group_read([(1, 132, 4), (3, 126, 2)], True)
        assert response.startswith(HEADER + b'\xfe')
        assert response[8:14] == b'\x00\x01\xff\xff\xfd\x00'
        assert response[16:20] == b'\x00\x03\xfd\xff'
        assert len(response) == 7 + number(response[5:7])
        assert crc16(response[:-2]) == number(response[-2:])
    finally:
        bus.close()
