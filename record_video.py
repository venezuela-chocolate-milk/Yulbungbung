"""최종 모델의 주행을 sumo-gui로 녹화해 mp4로 만든다 (마스킹 끔, 기준 교통량).

    python record_video.py                                   ← results/final_model/model.pt, 경로 north_south_left
    python record_video.py --route north_curve_uturn --fps 10

매 시뮬레이션 스텝(0.5 s)마다 화면을 한 장씩 저장하고 ffmpeg로 묶는다.
--fps 10이면 5배속 영상이 된다. 완주하지 못한 에피소드는 버리고 --tries 번까지 다시 찍는다.
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys

BASE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE)


def record_episode(model_path, route, frame_dir, size, view_w, max_steps=None):
    import traci
    from env.maps import make_env
    from test import load_agent
    from train import REWARD

    env, _ = make_env("roundabout", reward=REWARD, gui=True, gui_autostart=True)
    env.set_mask(False)
    env.gui_delay = 0.0
    env.gui_track_ego = False   # 확대 배율 대신 ego 중심 표시 범위(m)를 매 스텝 직접 지정
    # --window-size는 메뉴/툴바를 포함한 창 크기다. 지도 화면(캔버스)이 영상 크기보다 커야
    # 빈 부분에 화면 위쪽이 반복돼 찍히지 않으므로 세로 여유를 둔다.
    env.sumo_args = list(env.sumo_args) + ["--window-size", f"{size[0] + 40},{size[1] + 200}"]
    agent = load_agent(model_path, env.observation_space.shape[0], env.action_space.shape[0])
    try:
        obs, _ = env.reset(options={"route": route})
        view = traci.gui.getIDList()[0]
        view_h = view_w * size[1] / size[0]
        k = [0]
        # 11-24: 시뮬레이션 스텝마다 한 장. 주차 동작(sumo_env._park_ego)은 환경 한 스텝 안에서 여러 시뮬레이션 스텝을
        #   돌기 때문에, 환경 스텝마다 찍으면 칸에 넣었다 빼는 장면이 통째로 빠진다. (녹화만 바뀜, 주행은 그대로)
        orig_step = traci.simulationStep

        def step_and_shot(*a, **kw):
            r = orig_step(*a, **kw)
            if env.ego_id in traci.vehicle.getIDList():
                x, y = traci.vehicle.getPosition(env.ego_id)
                traci.gui.setBoundary(view, x - view_w / 2, y - view_h / 2, x + view_w / 2, y + view_h / 2)
            traci.gui.screenshot(view, os.path.join(frame_dir, f"f{k[0]:05d}.png"))
            k[0] += 1
            return r
        traci.simulationStep = step_and_shot
        n, info = 0, {}
        try:
            while True:
                obs, _, te, tr, info = env.step(agent.predict(obs, deterministic=True))
                n += 1
                if te or tr or (max_steps and n >= max_steps):
                    break
        finally:
            traci.simulationStep = orig_step
        result = ("arrived" if info.get("arrived") else "collided" if info.get("collided")
                  else "lost" if info.get("lost") else "red_violation" if info.get("red_violation")
                  else "timeout")
        return result, n
    finally:
        env.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--model", default=os.path.join(BASE, "results", "final_model", "model.pt"))
    ap.add_argument("--route", default="north_south_left")
    ap.add_argument("--fps", type=int, default=10, help="영상 fps (스텝 0.5 s → 10이면 5배속)")
    ap.add_argument("--view-width", type=float, default=200.0, help="화면 가로에 보일 범위 (m)")
    ap.add_argument("--max-steps", type=int, default=None, help="시험용: 이 스텝까지만 녹화")
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--tries", type=int, default=3)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    size = tuple(int(v) for v in args.size.split("x"))
    out = args.out or os.path.join(BASE, "results", "final_model", f"drive_{args.route}.mp4")
    frame_dir = out + ".frames"
    for t in range(args.tries):
        shutil.rmtree(frame_dir, ignore_errors=True)
        os.makedirs(frame_dir)
        result, steps = record_episode(args.model, args.route, frame_dir, size, args.view_width, args.max_steps)
        print(f"시도 {t + 1}: {result}, {steps} 스텝 ({steps * 0.5:.0f} s 주행)")
        if result == "arrived":
            break
    frames = sorted(glob.glob(os.path.join(frame_dir, "f*.png")))
    frames = [f for f in frames if os.path.getsize(f) > 0]
    # 캔버스 가운데(ego 추적 위치)를 영상 크기로 잘라낸다.
    vf = f"crop={size[0]}:{size[1]}:(iw-{size[0]})/2:(ih-{size[1]})/2"
    import imageio_ffmpeg
    src = [imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-framerate", str(args.fps),
           "-i", os.path.join(frame_dir, "f%05d.png"), "-frames:v", str(len(frames)), "-vf", vf]
    subprocess.run(src + ["-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out], check=True)
    webm = os.path.splitext(out)[0] + ".webm"
    subprocess.run(src + ["-c:v", "libvpx-vp9", "-b:v", "0", "-crf", "32", "-row-mt", "1",
                          "-pix_fmt", "yuv420p", webm], check=True)
    shutil.rmtree(frame_dir, ignore_errors=True)
    print(f"영상: {out} / {webm}  ({len(frames)} 프레임, {len(frames) / args.fps:.0f} s, 결과 {result})")


if __name__ == "__main__":
    main()
