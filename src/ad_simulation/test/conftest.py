"""ad_simulation SIL 테스트 공통 pytest fixture.

Gazebo headless 프로세스 관리 및 rclpy 테스트 노드를 제공한다.

사용법:
    pytest src/ad_simulation/test/ -v
"""

import os
import signal
import subprocess
import time
from typing import Generator, Optional

import pytest

# --------------------------------------------------------------------------- #
# rclpy / ROS2 메시지 임포트 (설치되어 있지 않으면 테스트 건너뜀)
# --------------------------------------------------------------------------- #
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False

# --------------------------------------------------------------------------- #
# 헬퍼: Gazebo 프로세스 관리
# --------------------------------------------------------------------------- #

def _find_world_sdf() -> str:
    """설치된 SDF 월드 파일 경로를 탐색한다.

    1) ament_index 경로 (colcon 빌드 후)
    2) 소스 디렉토리 기준 상대 경로 (개발 중)
    """
    # 빌드 환경에서 ament_index 사용 시도
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory('ad_simulation')
        sdf = os.path.join(pkg, 'worlds', 'agricultural_field.sdf')
        if os.path.isfile(sdf):
            return sdf
    except Exception:
        pass

    # 소스 디렉토리 기준 폴백
    here = os.path.dirname(os.path.abspath(__file__))
    sdf = os.path.join(here, '..', 'worlds', 'agricultural_field.sdf')
    sdf = os.path.normpath(sdf)
    if os.path.isfile(sdf):
        return sdf

    pytest.skip('agricultural_field.sdf 월드 파일을 찾을 수 없습니다.')
    return ''  # 도달하지 않음


# --------------------------------------------------------------------------- #
# Fixture: Gazebo headless 프로세스
# --------------------------------------------------------------------------- #

@pytest.fixture(scope='session')
def gazebo_process() -> Generator[Optional[subprocess.Popen], None, None]:
    """Headless Gazebo(gz sim) 프로세스를 시작하고 테스트 종료 시 정리한다.

    - ``GZ_SIM_HEADLESS=1`` 환경 변수로 GUI 없이 실행
    - 환경에 ``gz`` 명령이 없으면 fixture를 ``None``으로 반환하여
      mock 기반 테스트가 계속 실행되도록 한다.

    Yields:
        subprocess.Popen 또는 None (Gazebo가 설치되어 있지 않은 경우)
    """
    # gz sim 명령 존재 확인
    gz_cmd = 'gz'
    try:
        subprocess.run(
            [gz_cmd, 'sim', '--version'],
            capture_output=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        # Gazebo가 설치되어 있지 않음 — mock 테스트만 실행
        yield None
        return

    world_sdf = _find_world_sdf()
    model_dir = os.path.normpath(
        os.path.join(os.path.dirname(world_sdf), '..', 'models')
    )

    env = os.environ.copy()
    # 헤드리스 렌더링 (GPU 없는 CI 환경 대응)
    env['GZ_SIM_HEADLESS'] = '1'
    # 모델 리소스 경로
    existing = env.get('GZ_SIM_RESOURCE_PATH', '')
    env['GZ_SIM_RESOURCE_PATH'] = (
        model_dir + os.pathsep + existing if existing else model_dir
    )

    proc = subprocess.Popen(
        [gz_cmd, 'sim', '-r', '-s', world_sdf],  # -s: 서버 전용, -r: 즉시 실행
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Gazebo 서버가 준비될 때까지 대기 (최대 30초)
    startup_timeout = 30.0
    start_time = time.time()
    ready = False
    while time.time() - start_time < startup_timeout:
        if proc.poll() is not None:
            # 프로세스가 이미 종료됨
            break
        try:
            # gz topic -l 로 토픽 목록이 나오면 서버 준비 완료로 판단
            result = subprocess.run(
                [gz_cmd, 'topic', '-l'],
                capture_output=True,
                text=True,
                timeout=5,
                env=env,
            )
            if '/clock' in result.stdout:
                ready = True
                break
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        time.sleep(1.0)

    if not ready and proc.poll() is None:
        # 타임아웃이지만 프로세스는 살아있음 — 테스트 계속 진행
        pass

    yield proc

    # 정리: 프로세스 종료
    if proc.poll() is None:
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)


# --------------------------------------------------------------------------- #
# Fixture: ROS2 rclpy 테스트 노드
# --------------------------------------------------------------------------- #

@pytest.fixture(scope='function')
def ros2_node():
    """테스트용 rclpy 노드를 생성하고 테스트 종료 시 정리한다.

    rclpy가 설치되어 있지 않으면 테스트를 건너뛴다.

    Yields:
        tuple(Node, SingleThreadedExecutor)
    """
    if not HAS_RCLPY:
        pytest.skip('rclpy가 설치되어 있지 않습니다.')

    # rclpy 초기화 (이미 초기화되어 있으면 무시)
    try:
        rclpy.init()
    except RuntimeError:
        pass  # 이미 초기화됨

    node = rclpy.create_node('sil_test_node')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    yield node, executor

    # 정리
    node.destroy_node()
    executor.shutdown()
    try:
        rclpy.shutdown()
    except Exception:
        pass
