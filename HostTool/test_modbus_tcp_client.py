"""Modbus TCP 客户端帧校验和异常处理单元测试。

测试通过内存 Socket 替身构造正常及畸形 MBAP/PDU 响应，验证事务标识、协议标识、
单元标识、功能码、字节数和写响应回显检查，不需要连接真实视觉控制器。
"""

import struct
import unittest

from modbus_tcp_client import ModbusTCPClient, ModbusTCPError


class FakeSocket:
    """按请求动态生成响应数据的内存 Socket 替身。"""
    def __init__(self, response_factory):
        self._response_factory = response_factory
        self._rx = bytearray()

    def sendall(self, request):
        self._rx.extend(self._response_factory(bytes(request)))

    def recv(self, size):
        data = bytes(self._rx[:size])
        del self._rx[:size]
        return data


def response_for(request: bytes, pdu: bytes, *, protocol=0, unit=None) -> bytes:
    tid, _, _, request_unit = struct.unpack(">HHHB", request[:7])
    response_unit = request_unit if unit is None else unit
    return struct.pack(">HHHB", tid, protocol, len(pdu) + 1, response_unit) + pdu


class ModbusTCPClientTests(unittest.TestCase):
    def make_client(self, response_factory):
        client = ModbusTCPClient("127.0.0.1")
        client._sock = FakeSocket(response_factory)
        return client

    def test_reads_single_holding_register_as_u16(self):
        client = self.make_client(
            lambda req: response_for(req, bytes.fromhex("03 02 12 34")))
        self.assertEqual(client.read_holding_register(10), 0x1234)

    def test_rejects_wrong_protocol_id(self):
        client = self.make_client(
            lambda req: response_for(req, bytes.fromhex("03 02 00 01"), protocol=1))
        with self.assertRaisesRegex(ModbusTCPError, "协议 ID"):
            client.read_holding_register(0)

    def test_rejects_mismatched_function(self):
        client = self.make_client(
            lambda req: response_for(req, bytes.fromhex("04 02 00 01")))
        with self.assertRaisesRegex(ModbusTCPError, "功能码不匹配"):
            client.read_holding_register(0)

    def test_rejects_bad_fc03_byte_count(self):
        client = self.make_client(
            lambda req: response_for(req, bytes.fromhex("03 01 12")))
        with self.assertRaisesRegex(ModbusTCPError, "FC03"):
            client.read_holding_register(0)

    def test_rejects_wrong_fc06_echo(self):
        def response(req):
            pdu = bytearray(req[7:])
            pdu[-1] ^= 1
            return response_for(req, bytes(pdu))

        client = self.make_client(response)
        with self.assertRaisesRegex(ModbusTCPError, "FC06"):
            client.write_single_register(20, 7)

    def test_rejects_wrong_fc16_echo(self):
        client = self.make_client(
            lambda req: response_for(req, bytes.fromhex("10 00 64 00 01")))
        with self.assertRaisesRegex(ModbusTCPError, "FC16"):
            client.write_multiple_registers(100, [1, 2])


if __name__ == "__main__":
    unittest.main()
