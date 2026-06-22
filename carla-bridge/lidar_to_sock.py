import carla, socket, struct, queue, random

HOST, PORT = "0.0.0.0", 9871
client = carla.Client("localhost", 2000); client.set_timeout(30)
world = client.get_world()
print("connected, map:", world.get_map().name)

# recover server to async in case a prior run left it frozen in sync mode
s0 = world.get_settings()
s0.synchronous_mode = False
s0.fixed_delta_seconds = None
world.apply_settings(s0)

for a in world.get_actors().filter("vehicle.*"):
    try: a.destroy()
    except Exception: pass

bp = world.get_blueprint_library()
vbp = bp.filter("vehicle.tesla.model3")[0]
sps = world.get_map().get_spawn_points(); random.shuffle(sps)
ego = None
for sp in sps:
    ego = world.try_spawn_actor(vbp, sp)
    if ego: break
if ego is None: raise RuntimeError("no free spawn")
print("ego spawned:", ego.id)

lbp = bp.find("sensor.lidar.ray_cast")
lbp.set_attribute("range", "50")
lbp.set_attribute("channels", "32")
lbp.set_attribute("points_per_second", "120000")
lbp.set_attribute("rotation_frequency", "10")
lidar = world.spawn_actor(lbp, carla.Transform(carla.Location(z=2.0)), attach_to=ego)
q = queue.Queue()
lidar.listen(lambda m: q.put(bytes(m.raw_data)))
print("lidar attached")

# NOW switch to synchronous mode (after all spawns)
tm = client.get_trafficmanager(8000)
tm.set_synchronous_mode(True)
ego.set_autopilot(True, 8000)
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.1
world.apply_settings(settings)

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind((HOST, PORT)); srv.listen(1)
print("waiting for consumer on :%d ..." % PORT)
conn, addr = srv.accept()
print("consumer connected:", addr)

f = 0
try:
    while True:
        world.tick()
        try: data = q.get(timeout=2.0)
        except queue.Empty: continue
        conn.sendall(struct.pack(">I", len(data)) + data)
        f += 1
        if f <= 3 or f % 20 == 0:
            print("sent frame %d: %d points" % (f, len(data)//16))
except (KeyboardInterrupt, BrokenPipeError, ConnectionResetError):
    pass
finally:
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    tm.set_synchronous_mode(False)
    try: lidar.destroy(); ego.destroy()
    except Exception: pass
    print("cleaned up")
