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

"""Independent wire-level assertions shared by simulator and integration tests."""

from dxl_sim import crc16, HEADER, number


def status_packets(reply):
    """Split actual wire bytes without repairing malformed framing or CRCs."""
    packets = []
    while reply:
        assert reply.startswith(HEADER) and len(reply) >= 11
        length = number(reply[5:7]) + 7
        assert len(reply) >= length
        packets.append(reply[:length])
        reply = reply[length:]
    return packets


def assert_transient_reply(fault, fast, normal, injected):
    """Require the requested fault in the actual bytes, independent of event records."""
    packets = status_packets(normal)
    assert [packet[4] for packet in packets] == ([254] if fast else list(range(1, 7)))
    assert all(crc16(packet[:-2]) == number(packet[-2:]) for packet in packets)
    if fault == 'drop' or (fault == 'silent' and fast):
        assert injected == b'', f'{fault}: expected no reply, got {injected.hex()}'
    elif fault == 'crc':
        assert injected == normal[:-1] + bytes([normal[-1] ^ 1]), 'CRC fault had no effect'
        corrupt = status_packets(injected)
        assert crc16(corrupt[-1][:-2]) != number(corrupt[-1][-2:])
        assert all(crc16(packet[:-2]) == number(packet[-2:]) for packet in corrupt[:-1])
    else:
        assert fault == 'silent' and not fast
        expected = b''.join(packet for packet in packets if packet[4] != 3)
        assert injected == expected, 'silent servo must omit only ID 3 status'
        assert [packet[4] for packet in status_packets(injected)] == [1, 2, 4, 5, 6]
