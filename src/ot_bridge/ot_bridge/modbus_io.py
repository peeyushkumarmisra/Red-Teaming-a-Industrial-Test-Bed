from ot_bridge.modbus_map import (COIL_COUNT, CellCommand, COMMAND_REGISTERS, STATUS_BASE,
                                  STATUS_HALTED, STATUS_REFERENCE_SEQ, STATUS_REGISTERS,
                                  STATUS_STEP_ACK)

FC_COIL = 1
FC_DISCRETE = 2
FC_HOLDING = 3
FC_INPUT = 4

HOLDING_SIZE = 64
INPUT_SIZE = 16


def new_context():
    from pymodbus.datastore import ModbusSequentialDataBlock
    try:
        from pymodbus.datastore import ModbusSlaveContext as DeviceContext
    except ImportError:
        from pymodbus.datastore import ModbusDeviceContext as DeviceContext
    return DeviceContext(
        co=ModbusSequentialDataBlock(0x00, [0] * COIL_COUNT),
        di=ModbusSequentialDataBlock(0x00, [0] * COIL_COUNT),
        hr=ModbusSequentialDataBlock(0x00, [0] * HOLDING_SIZE),
        ir=ModbusSequentialDataBlock(0x00, [0] * INPUT_SIZE))


class Store:

    def read_holding(self, address, count):
        raise NotImplementedError

    def write_holding(self, address, values):
        raise NotImplementedError

    def read_coil(self, address):
        raise NotImplementedError

    def write_coil(self, address, value):
        raise NotImplementedError

    def read_input(self, address, count):
        raise NotImplementedError

    def write_input(self, address, values):
        raise NotImplementedError


class ContextStore(Store):

    def __init__(self, context=None):
        self.context = context if context is not None else new_context()

    def read_holding(self, address, count):
        return list(self.context.getValues(FC_HOLDING, address, count=count))

    def write_holding(self, address, values):
        self.context.setValues(FC_HOLDING, address, list(values))

    def read_coil(self, address):
        return bool(self.context.getValues(FC_COIL, address, count=1)[0])

    def write_coil(self, address, value):
        self.context.setValues(FC_COIL, address, [1 if value else 0])

    def read_input(self, address, count):
        return list(self.context.getValues(FC_INPUT, address, count=count))

    def write_input(self, address, values):
        self.context.setValues(FC_INPUT, address, list(values))


class LocalStore(Store):

    def __init__(self):
        self._hr = [0] * HOLDING_SIZE
        self._co = [0] * COIL_COUNT
        self._ir = [0] * INPUT_SIZE

    def read_holding(self, address, count):
        return list(self._hr[address:address + count])

    def write_holding(self, address, values):
        self._hr[address:address + len(values)] = [int(v) & 0xFFFF for v in values]

    def read_coil(self, address):
        return bool(self._co[address])

    def write_coil(self, address, value):
        self._co[address] = 1 if value else 0

    def read_input(self, address, count):
        return list(self._ir[address:address + count])

    def write_input(self, address, values):
        self._ir[address:address + len(values)] = [int(v) & 0xFFFF for v in values]


def _call(method, *args, device_id, **kwargs):
    try:
        return method(*args, device_id=device_id, **kwargs)
    except TypeError:
        try:
            return method(*args, slave=device_id, **kwargs)
        except TypeError:
            return method(*args, unit=device_id, **kwargs)


class TcpStore(Store):

    def __init__(self, host, port=502, device_id=1, timeout=1.0):
        from pymodbus.client import ModbusTcpClient
        self.client = ModbusTcpClient(host, port=port, timeout=timeout)
        self.device_id = device_id

    def connect(self):
        return self.client.connect()

    def close(self):
        self.client.close()

    def read_holding(self, address, count):
        rr = _call(self.client.read_holding_registers, address, count=count,
                   device_id=self.device_id)
        if rr.isError():
            raise IOError(f'read_holding {address}+{count}: {rr}')
        return list(rr.registers)

    def write_holding(self, address, values):
        rr = _call(self.client.write_registers, address, list(values), device_id=self.device_id)
        if rr.isError():
            raise IOError(f'write_holding {address}: {rr}')

    def read_coil(self, address):
        rr = _call(self.client.read_coils, address, count=1, device_id=self.device_id)
        if rr.isError():
            raise IOError(f'read_coil {address}: {rr}')
        return bool(rr.bits[0])

    def write_coil(self, address, value):
        rr = _call(self.client.write_coil, address, bool(value), device_id=self.device_id)
        if rr.isError():
            raise IOError(f'write_coil {address}: {rr}')

    def read_input(self, address, count):
        rr = _call(self.client.read_input_registers, address, count=count,
                   device_id=self.device_id)
        if rr.isError():
            raise IOError(f'read_input {address}+{count}: {rr}')
        return list(rr.registers)


def write_command(store, command):
    store.write_holding(0, command.to_registers())


def read_command(store):
    return CellCommand.from_registers(store.read_holding(0, COMMAND_REGISTERS))


def write_status(store, halted, step_ack, reference_seq):
    values = [0] * STATUS_REGISTERS
    values[STATUS_HALTED] = 1 if halted else 0
    values[STATUS_STEP_ACK] = int(step_ack) & 0xFFFF
    values[STATUS_REFERENCE_SEQ] = int(reference_seq) & 0xFFFF
    store.write_holding(STATUS_BASE, values)


def read_status(store):
    values = store.read_holding(STATUS_BASE, STATUS_REGISTERS)
    return {'halted': bool(values[STATUS_HALTED]), 'step_ack': values[STATUS_STEP_ACK],
            'reference_seq': values[STATUS_REFERENCE_SEQ]}
