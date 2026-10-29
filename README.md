# Foosball_CU

A robot that plays foosball, built at Columbia University. Reinforcement-learning agents are trained in simulation and then drive motorized rods on a real table.

<p align="center"><img src="https://thakur-sachin.github.io/sachin_portfolio/img/foosball-cad.jpg" alt="CAD model of the foosball table with linear and rotary actuators on every rod" width="600"></p>

The simulation work was presented as *End-to-End Deep Reinforcement Learning Platform for Competitive Physical Dynamic Systems* (NeurIPS 2024).

## Repository layout

The project was split across three teams: mechanical design, simulation, and AI agents.

| Path | Contents |
|---|---|
| `CAD_design/` | SolidWorks parts and assembly for the table, rods, actuator mounts, and ball-return mechanism; Fusion 360 full assembly |
| `foosball_sim/` | MuJoCo models of the table (`v1`, `v2`) with inertia and STL-centering helpers |
| `Isaac_Lab_Files/` | Isaac Lab environment for the table (`foosball2/foosball_env.py`) |
| `ai_agents/` | Gym environments (single-rod and full-table protagonist/antagonist), SAC agents, and training engines |
| `servo_control/` | Setup for the Teknic ClearPath servos (sFoundation SDK and Exar USB driver) |
| `sac_agent_entry.py`, `sac_agent_entry_v2.py` | Training and evaluation entry points |

## Training an agent

```bash
pip install stable-baselines3 mujoco gymnasium
python sac_agent_entry_v2.py        # train
python sac_agent_entry_v2.py --test # evaluate the saved model
```

Models are written to `./models` and logs to `./logs`. `v2` trains soft actor-critic agents with protagonist–antagonist self-play against frozen copies of the best previous model.

## Driving the real table

See `servo_control/how_to_compile` to install the Exar USB-serial driver and build Teknic's sFoundation library for the ClearPath motors.

## License

See [LICENSE](LICENSE).
