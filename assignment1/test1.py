
import irsim
from custom_behavior_methods_1 import metrics_p

env = irsim.make("test1.yaml")
env.load_behavior("custom_behavior_methods_1")

for _ in range(500):
    env.step()
    env.render(0.03)
    if env.done():
        break

metrics_p()
env.end(3)








