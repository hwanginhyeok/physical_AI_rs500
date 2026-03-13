"""YOLOv8 모델 관리자.

MX550 (2GB VRAM) 환경에 최적화된 경량 모델 로딩 및 관리.
"""

import os
from pathlib import Path
from typing import Optional, Dict, Tuple
import torch
import numpy as np


class ModelManager:
    """YOLOv8 모델 로딩 및 관리.
    
    자동으로 모델을 다운로드하고 CUDA/CPU 메모리를 효율적으로 관리.
    """
    
    # 모델 설정 (모델명: (파일크기_MB, 추론시간_ms_예상, 권장배치))
    MODEL_CONFIGS = {
        "yolov8n": {"size_mb": 6, "latency_ms": 5, "task": "detect"},
        "yolov8n-seg": {"size_mb": 7, "latency_ms": 15, "task": "segment"},
        "yolov8s-seg": {"size_mb": 23, "latency_ms": 25, "task": "segment"},
    }
    
    def __init__(self, model_dir: Optional[str] = None, device: Optional[str] = None):
        """초기화.
        
        Args:
            model_dir: 모델 저장 디렉토리. None이면 패키지 내 models/ 사용
            device: 'cuda', 'cpu', 또는 None(자동)
        """
        if model_dir is None:
            # 패키지 디렉토리 기준
            pkg_dir = Path(__file__).parent.parent
            self.model_dir = pkg_dir / "models"
        else:
            self.model_dir = Path(model_dir)
        
        self.model_dir.mkdir(parents=True, exist_ok=True)
        
        # 디바이스 설정
        if device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device
        
        # VRAM 확인 및 경고
        if self.device == "cuda":
            self._check_vram()
        
        # 로드된 모델 캐시
        self._models: Dict[str, any] = {}
        
        print(f"[ModelManager] Device: {self.device}")
        print(f"[ModelManager] Model dir: {self.model_dir}")
    
    def _check_vram(self):
        """VRAM 확인 및 경고."""
        try:
            props = torch.cuda.get_device_properties(0)
            total_vram = props.total_memory / (1024**3)  # GB
            print(f"[ModelManager] GPU: {props.name}")
            print(f"[ModelManager] VRAM: {total_vram:.1f} GB")
            
            if total_vram < 3:
                print("[ModelManager] ⚠️ Low VRAM detected. Using single precision.")
                # MX550 등 일부 GPU는 FP16 미지원 - FP32 사용
                self.use_half = False
            else:
                self.use_half = False
                
        except Exception as e:
            print(f"[ModelManager] Could not check VRAM: {e}")
            self.use_half = False
    
    def load_model(self, model_name: str, force_reload: bool = False):
        """모델 로드.
        
        Args:
            model_name: 'yolov8n', 'yolov8n-seg', 등
            force_reload: True면 캐시 무시하고 재로드
            
        Returns:
            로드된 YOLO 모델
        """
        if not force_reload and model_name in self._models:
            return self._models[model_name]
        
        try:
            from ultralytics import YOLO
        except ImportError:
            print("[ModelManager] Installing ultralytics...")
            import subprocess
            subprocess.run(["pip", "install", "-q", "ultralytics"], check=True)
            from ultralytics import YOLO
        
        model_path = self.model_dir / f"{model_name}.pt"
        
        # 모델 다운로드
        if not model_path.exists():
            print(f"[ModelManager] Downloading {model_name}...")
            model = YOLO(model_name)
            # ultralytics는 자동으로 캐시함
        else:
            print(f"[ModelManager] Loading {model_name} from {model_path}")
            model = YOLO(str(model_path))
        
        # 디바이스 이동 및 최적화
        if self.device == "cuda" and hasattr(self, 'use_half') and self.use_half:
            model.half()  # FP16
        
        # 웜업 (첫 추론은 느림)
        dummy_input = torch.zeros(1, 3, 640, 640)
        if self.device == "cuda":
            dummy_input = dummy_input.cuda()
        if hasattr(self, 'use_half') and self.use_half:
            dummy_input = dummy_input.half()
        
        with torch.no_grad():
            _ = model.predict(dummy_input, verbose=False)
        
        self._models[model_name] = model
        print(f"[ModelManager] {model_name} loaded successfully")
        
        return model
    
    def get_model(self, model_name: str) -> any:
        """캐시된 모델 반환. 없으면 로드."""
        if model_name not in self._models:
            return self.load_model(model_name)
        return self._models[model_name]
    
    def unload_model(self, model_name: str):
        """모델 메모리에서 해제."""
        if model_name in self._models:
            del self._models[model_name]
            if self.device == "cuda":
                torch.cuda.empty_cache()
            print(f"[ModelManager] {model_name} unloaded")
    
    def list_available_models(self) -> list:
        """사용 가능한 모델 목록."""
        return list(self.MODEL_CONFIGS.keys())
    
    def get_model_info(self, model_name: str) -> Optional[dict]:
        """모델 정보 반환."""
        return self.MODEL_CONFIGS.get(model_name)
    
    def get_memory_usage(self) -> Dict[str, float]:
        """현재 메모리 사용량."""
        info = {"loaded_models": len(self._models)}
        
        if self.device == "cuda":
            info["allocated_gb"] = torch.cuda.memory_allocated() / (1024**3)
            info["reserved_gb"] = torch.cuda.memory_reserved() / (1024**3)
        
        return info
