# Floyd-IsaacLab

Isaac Lab training configuration for Floyd, a BDX-style bipedal robot.

## Robot
- 8 DOF (no hip yaw): Hip Roll, Hip Pitch, Knee Pitch, Ankle Pitch per leg
- RS03 actuators: Hip Roll, Hip Pitch, Knee Pitch
- RS02 actuators: Ankle Pitch
- USD: `data/Robots/Floyd/Floyd-BDX.usd`

## Training
```bash
conda activate env_isaaclab
cd C:\Users\logan\IsaacLab
python C:\Users\logan\Floyd-IsaacLab\scripts\train.py --task Floyd-Velocity-Flat-v0 --num_envs 4096
```

## Play
```bash
python C:\Users\logan\Floyd-IsaacLab\scripts\play.py --task Floyd-Velocity-Flat-Play-v0 --num_envs 50
```
