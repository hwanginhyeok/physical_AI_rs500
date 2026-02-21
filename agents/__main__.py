"""에이전트 시스템 CLI 진입점.

Usage:
    python -m agents                                # 대화형 모드
    python -m agents model vehicle --mass 800       # 차량 모델 수정
    python -m agents model vehicle --info            # 차량 정보 조회
    python -m agents model world --add-obstacle box  # 장애물 추가
    python -m agents model physics --step-size 0.002 # 물리 설정
    python -m agents research "궤도차량 자율주행"      # 자료 조사
    python -m agents status                          # 프로젝트 현황
"""

from .main import main

main()
