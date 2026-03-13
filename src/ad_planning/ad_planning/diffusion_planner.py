"""Diffusion-based Trajectory Planner.

Lightweight diffusion model for agricultural path planning.
Optimized for MX550 (2GB VRAM).
"""

import torch
import torch.nn as nn
import numpy as np
from typing import List, Optional, Tuple
from dataclasses import dataclass


@dataclass
class PlannerConfig:
    """Diffusion Planner 설정."""
    # 궤적 설정
    num_waypoints: int = 20          # 경로 점 개수
    waypoint_dim: int = 3            # (x, y, yaw)
    prediction_horizon: float = 10.0  # s
    
    # 모델 설정 (경량화)
    hidden_dim: int = 128
    num_layers: int = 4
    time_embed_dim: int = 64
    
    # Diffusion 설정
    num_diffusion_steps: int = 20    # 빠른 샘플링
    beta_start: float = 0.0001
    beta_end: float = 0.02
    
    # Conditioning
    condition_dim: int = 32          # perception features
    
    # 장치
    device: str = "cuda"


class SinusoidalTimeEmbedding(nn.Module):
    """시간 임베딩 (Transformer 스타일)."""
    
    def __init__(self, dim: int):
        super().__init__()
        self.dim = dim
    
    def forward(self, time: torch.Tensor) -> torch.Tensor:
        device = time.device
        half_dim = self.dim // 2
        embeddings = np.log(10000) / (half_dim - 1)
        embeddings = torch.exp(torch.arange(half_dim, device=device) * -embeddings)
        embeddings = time[:, None] * embeddings[None, :]
        embeddings = torch.cat([embeddings.sin(), embeddings.cos()], dim=-1)
        return embeddings


class DiffusionPlannerModel(nn.Module):
    """경량 Diffusion 기반 경로 생성 모델.
    
    MLP 기반으로 U-Net보다 가볍고 빠름.
    입력: 노이즈 궤적 + 시간 + 조건
    출력: 노이즈 제거된 궤적
    """
    
    def __init__(self, config: PlannerConfig):
        super().__init__()
        self.config = config
        
        # 시간 임베딩
        self.time_embed = SinusoidalTimeEmbedding(config.time_embed_dim)
        self.time_mlp = nn.Sequential(
            nn.Linear(config.time_embed_dim, config.time_embed_dim),
            nn.SiLU(),
            nn.Linear(config.time_embed_dim, config.time_embed_dim),
        )
        
        # 조건 임베딩 (perception features)
        self.condition_embed = nn.Sequential(
            nn.Linear(config.condition_dim, config.hidden_dim),
            nn.SiLU(),
        )
        
        # 입력 투영
        input_dim = config.num_waypoints * config.waypoint_dim
        self.input_proj = nn.Linear(input_dim, config.hidden_dim)
        
        # MLP 블록들
        layers = []
        for i in range(config.num_layers):
            layers.append(nn.Linear(config.hidden_dim, config.hidden_dim))
            layers.append(nn.LayerNorm(config.hidden_dim))
            layers.append(nn.SiLU())
            layers.append(nn.Dropout(0.1))
        
        self.backbone = nn.Sequential(*layers)
        
        # 시간 및 조건 결합
        self.fusion = nn.Sequential(
            nn.Linear(config.hidden_dim + config.time_embed_dim, config.hidden_dim),
            nn.SiLU(),
        )
        
        # 출력 투영
        self.output_proj = nn.Linear(config.hidden_dim, input_dim)
    
    def forward(
        self,
        x: torch.Tensor,           # (B, num_waypoints, waypoint_dim) - 노이즈 궤적
        t: torch.Tensor,           # (B,) - diffusion timestep
        condition: torch.Tensor    # (B, condition_dim) - perception features
    ) -> torch.Tensor:
        """Forward pass.
        
        Returns:
            denoised trajectory (B, num_waypoints, waypoint_dim)
        """
        B = x.shape[0]
        
        # Flatten trajectory
        x_flat = x.reshape(B, -1)  # (B, num_waypoints * waypoint_dim)
        
        # 시간 임베딩
        t_embed = self.time_mlp(self.time_embed(t))  # (B, time_embed_dim)
        
        # 조건 임베딩
        c_embed = self.condition_embed(condition)    # (B, hidden_dim)
        
        # 입력 투영
        h = self.input_proj(x_flat)  # (B, hidden_dim)
        h = h + c_embed              # residual conditioning
        
        # Backbone
        h = self.backbone(h)         # (B, hidden_dim)
        
        # 시간 결합
        h = torch.cat([h, t_embed], dim=-1)
        h = self.fusion(h)
        
        # 출력
        out = self.output_proj(h)    # (B, num_waypoints * waypoint_dim)
        out = out.reshape(B, self.config.num_waypoints, self.config.waypoint_dim)
        
        return out


class DiffusionPlanner:
    """Diffusion 기반 경로 생성기.
    
    농업용 경로 계획을 위한 학습 기반 생성 모델.
    """
    
    def __init__(self, config: Optional[PlannerConfig] = None):
        """초기화."""
        self.config = config or PlannerConfig()
        self.device = torch.device(self.config.device if torch.cuda.is_available() else "cpu")
        
        # 모델 생성
        self.model = DiffusionPlannerModel(self.config).to(self.device)
        
        # Diffusion 스케줄 (DDPM)
        self.betas = torch.linspace(
            self.config.beta_start,
            self.config.beta_end,
            self.config.num_diffusion_steps,
            device=self.device
        )
        self.alphas = 1.0 - self.betas
        self.alphas_cumprod = torch.cumprod(self.alphas, dim=0)
        self.alphas_cumprod_prev = torch.cat([
            torch.tensor([1.0], device=self.device),
            self.alphas_cumprod[:-1]
        ])
        
        # 사전 계산
        self.sqrt_alphas_cumprod = torch.sqrt(self.alphas_cumprod)
        self.sqrt_one_minus_alphas_cumprod = torch.sqrt(1.0 - self.alphas_cumprod)
        
        print(f"[DiffusionPlanner] Initialized on {self.device}")
        print(f"  Parameters: {sum(p.numel() for p in self.model.parameters()):,}")
        print(f"  Diffusion steps: {self.config.num_diffusion_steps}")
        print(f"  Trajectory shape: ({self.config.num_waypoints}, {self.config.waypoint_dim})")
    
    def encode_condition(self, perception_features) -> torch.Tensor:
        """PerceptionFeatures를 조건 벡터로 인코딩."""
        # 간단한 수동 인코딩 (추후 learned encoder로 대체 가능)
        # terrain_type, obstacle_count, crop_row_count 등을 수치화
        
        condition = torch.zeros(self.config.condition_dim, device=self.device)
        
        # terrain encoding (one-hot 스타일)
        terrain_map = {
            'crop_field': 0,
            'grass': 1,
            'dirt_road': 2,
            'paved': 3,
            'mud': 4,
            'obstacle': 5,
        }
        terrain_idx = terrain_map.get(perception_features.terrain_type.value, 0)
        condition[terrain_idx] = 1.0
        
        # obstacle density
        condition[6] = min(len(perception_features.obstacles) / 10.0, 1.0)
        
        # crop row confidence
        condition[7] = perception_features.crop_row_confidence
        
        # slope
        condition[8] = perception_features.slope_gradient
        
        # overall confidence
        condition[9] = perception_features.overall_confidence
        
        return condition
    
    @torch.no_grad()
    def generate(
        self,
        perception_features,
        current_pose,
        current_speed: float,
        num_samples: int = 3,
        guidance_scale: float = 1.0
    ) -> List[dict]:
        """경로 생성.
        
        Args:
            perception_features: PerceptionFeatures
            current_pose: 현재 위치
            current_speed: 현재 속도
            num_samples: 생성할 경로 수 (multi-modal)
            guidance_scale: classifier-free guidance scale
            
        Returns:
            경로 리스트 (각 경로는 waypoints 리스트)
        """
        self.model.eval()
        
        # 조건 인코딩
        condition = self.encode_condition(perception_features)
        condition = condition.unsqueeze(0).repeat(num_samples, 1)  # (num_samples, condition_dim)
        
        # 초기 노이즈
        shape = (num_samples, self.config.num_waypoints, self.config.waypoint_dim)
        x = torch.randn(shape, device=self.device)
        
        # DDIM 샘플링 (빠른 샘플링)
        for i in reversed(range(self.config.num_diffusion_steps)):
            t = torch.full((num_samples,), i, device=self.device, dtype=torch.long)
            
            # 노이즈 예측
            predicted_noise = self.model(x, t.float(), condition)
            
            # DDIM 업데이트
            alpha_t = self.alphas_cumprod[i]
            alpha_t_prev = self.alphas_cumprod_prev[i]
            
            # x0 예측
            x0_pred = (x - torch.sqrt(1 - alpha_t) * predicted_noise) / torch.sqrt(alpha_t)
            
            if i > 0:
                # 이전 단계로 이동
                x = torch.sqrt(alpha_t_prev) * x0_pred + torch.sqrt(1 - alpha_t_prev) * predicted_noise
            else:
                x = x0_pred
        
        # 후처리: 상대 좌표를 절대 좌표로 변환
        trajectories = []
        for i in range(num_samples):
            waypoints = self._decode_trajectory(
                x[i].cpu().numpy(),
                current_pose,
                current_speed
            )
            trajectories.append({
                'waypoints': waypoints,
                'confidence': 1.0 - i * 0.1,  # 첫 번째가 가장 신뢰도 높음
            })
        
        return trajectories
    
    def _decode_trajectory(
        self,
        trajectory_array: np.ndarray,
        current_pose,
        current_speed: float
    ) -> List[dict]:
        """생성된 배열을 실제 경로로 디코딩."""
        waypoints = []
        
        # 현재 위치 기준으로 변환
        x0, y0, yaw0 = current_pose.x, current_pose.y, current_pose.yaw
        
        # trajectory_array: (num_waypoints, 3) - 상대 좌표 (dx, dy, dyaw)
        for i, (dx, dy, dyaw) in enumerate(trajectory_array):
            # 상대 → 절대 변환
            # 회전 행렬 적용
            cos_yaw = np.cos(yaw0)
            sin_yaw = np.sin(yaw0)
            
            x = x0 + dx * cos_yaw - dy * sin_yaw
            y = y0 + dx * sin_yaw + dy * cos_yaw
            yaw = yaw0 + dyaw
            
            # 속도 프로파일 (간단한 감속/가속)
            progress = i / len(trajectory_array)
            velocity = current_speed * (1.0 - progress * 0.3)  # 점진적 감속
            
            waypoints.append({
                'x': float(x),
                'y': float(y),
                'yaw': float(yaw),
                'velocity': float(velocity),
                'timestamp': i * (self.config.prediction_horizon / self.config.num_waypoints)
            })
        
        return waypoints
    
    def save_model(self, path: str):
        """모델 저장."""
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'config': self.config,
        }, path)
        print(f"[DiffusionPlanner] Model saved to {path}")
    
    def load_model(self, path: str):
        """모델 로드."""
        checkpoint = torch.load(path, map_location=self.device)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        print(f"[DiffusionPlanner] Model loaded from {path}")


# 간단한 테스트
if __name__ == "__main__":
    import sys
    sys.path.insert(0, '/home/gint_pcd/projects/자율주행프로젝트_work/src/ad_core')
    
    from ad_core.hybrid_e2e_types import PerceptionFeatures, TerrainClass
    from ad_core.datatypes import Pose2D
    
    print("=== Diffusion Planner 테스트 ===")
    
    # Planner 생성
    config = PlannerConfig(device="cpu", num_diffusion_steps=10)  # 빠른 테스트
    planner = DiffusionPlanner(config)
    
    # 테스트 입력
    perception = PerceptionFeatures(
        terrain_type=TerrainClass.CROP_FIELD,
        crop_row_confidence=0.8,
        overall_confidence=0.9
    )
    current_pose = Pose2D(x=0, y=0, yaw=0)
    
    # 경로 생성
    print("\\n경로 생성 중...")
    import time
    start = time.time()
    trajectories = planner.generate(perception, current_pose, current_speed=1.0, num_samples=3)
    elapsed = time.time() - start
    
    print(f"생성 시간: {elapsed*1000:.1f}ms")
    print(f"생성된 경로 수: {len(trajectories)}")
    
    for i, traj in enumerate(trajectories):
        print(f"\\n경로 {i+1} (confidence: {traj['confidence']:.2f}):")
        waypoints = traj['waypoints']
        print(f"  waypoints: {len(waypoints)}")
        print(f"  start: ({waypoints[0]['x']:.2f}, {waypoints[0]['y']:.2f})")
        print(f"  end: ({waypoints[-1]['x']:.2f}, {waypoints[-1]['y']:.2f})")
    
    print("\\n✅ 테스트 완료!")
