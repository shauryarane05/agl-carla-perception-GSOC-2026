import carla, time, threading
c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world()
print("connected, map:", w.get_map().name)
bp = w.get_blueprint_library()
vbp = bp.filter("vehicle.tesla.model3")[0]
sps = w.get_map().get_spawn_points()
veh = None
for sp in sps:
    veh = w.try_spawn_actor(vbp, sp)
    if veh: break
print("vehicle spawned:", veh.id)
lbp = bp.find("sensor.lidar.ray_cast")
lbp.set_attribute("range", "50")
lbp.set_attribute("channels", "32")
lbp.set_attribute("points_per_second", "100000")
lbp.set_attribute("rotation_frequency", "10")
import carla as _c
tf = _c.Transform(_c.Location(z=2.0))
lidar = w.spawn_actor(lbp, tf, attach_to=veh)
stats = {"frames":0, "pts":0, "t0":None}
def cb(m):
    if stats["t0"] is None: stats["t0"]=time.time()
    stats["frames"]+=1; stats["pts"]+=len(m)
    if stats["frames"]<=3 or stats["frames"]%10==0:
        print("  frame %d: %d points" % (stats["frames"], len(m)))
lidar.listen(cb)
print("listening for LiDAR for 15s...")
time.sleep(15)
lidar.stop()
dt = (time.time()-stats["t0"]) if stats["t0"] else 0
print("=== RESULT ===")
print("frames:", stats["frames"], "total points:", stats["pts"])
if dt>0: print("effective rate: %.2f Hz, avg %.0f pts/frame" % (stats["frames"]/dt, stats["pts"]/max(stats["frames"],1)))
lidar.destroy(); veh.destroy(); print("cleaned up")
