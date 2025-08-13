import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

void main() => runApp(const App());

class App extends StatelessWidget {
  const App({super.key});
  @override
  Widget build(BuildContext context) {
    return const MaterialApp(debugShowCheckedModeBanner: false, home: Home());
  }
}

class Home extends StatefulWidget {
  const Home({super.key});
  @override
  State<Home> createState() => _HomeState();
}

class _HomeState extends State<Home> {
  final host = TextEditingController(text: "192.168.1.10"); 
  final port = TextEditingController(text: "9090");
  WebSocketChannel? ch;
  String st = "disconnected";
  String joints = "-";
  double lx = 0.0; // linear.x
  double az = 0.0; // angular.z

  void connect() {
    final url = Uri.parse("ws://${host.text}:${port.text}");
    ch = WebSocketChannel.connect(url);
    setState(() => st = "connected");
    ch!.sink.add(jsonEncode({
      "op": "subscribe",
      "topic": "/joint_states",
      "type": "sensor_msgs/msg/JointState",
      "throttle_rate": 200
    }));
    ch!.stream.listen((m) {
      final v = jsonDecode(m);
      if (v is Map && v["topic"] == "/joint_states") {
        final n = (v["msg"]["name"] as List).cast<dynamic>().join(",");
        final p = (v["msg"]["position"] as List).cast<num>();
        joints = "$n | ${p.map((e)=>e.toStringAsFixed(2)).join(",")}";
        setState(() {});
      }
    }, onError: (_) => setState(() => st = "error"),
       onDone: () => setState(() => st = "closed"));
  }

  void disconnect() {
    ch?.sink.close();
    ch = null;
    setState(() => st = "disconnected");
  }

  void sendTwist() {
    if (ch == null) return;
    ch!.sink.add(jsonEncode({
      "op": "publish",
      "topic": "/cmd_vel",
      "type": "geometry_msgs/msg/Twist",
      "msg": {
        "linear": {"x": lx, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": az}
      }
    }));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("Week5 ROS Control")),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          Row(children: [
            Expanded(child: TextField(controller: host, decoration: const InputDecoration(labelText: "ROS Host"))),
            const SizedBox(width: 8),
            SizedBox(width: 90, child: TextField(controller: port, decoration: const InputDecoration(labelText: "Port")))
          ]),
          const SizedBox(height: 8),
          Row(children: [
            ElevatedButton(onPressed: connect, child: const Text("Connect")),
            const SizedBox(width: 8),
            ElevatedButton(onPressed: disconnect, child: const Text("Disconnect")),
            const SizedBox(width: 12),
            Text("Status: $st")
          ]),
          const Divider(height: 24),
          const Text("Linear X"), Slider(value: lx, onChanged: (v){ lx=v; setState((){}); }, min: -0.8, max: 0.8),
          const Text("Angular Z"), Slider(value: az, onChanged: (v){ az=v; setState((){}); }, min: -1.5, max: 1.5),
          Row(children: [
            ElevatedButton(onPressed: sendTwist, child: const Text("Send /cmd_vel")),
            const SizedBox(width: 12),
            ElevatedButton(onPressed: () { lx=0; az=0; setState((){}); sendTwist(); }, child: const Text("Stop"))
          ]),
          const Divider(height: 24),
          const Text("Joint States"),
          Text(joints, maxLines: 2, overflow: TextOverflow.ellipsis),
        ]),
      ),
    );
  }
}
