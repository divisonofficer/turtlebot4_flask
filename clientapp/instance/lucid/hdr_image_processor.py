"""
HDR Image Processing utilities for high-gain burst mode
독립적인 이미지 필터링 및 합성 처리 모듈
"""

import numpy as np
from typing import List, Tuple, Dict, Any
from lucid_cam import LucidImage


def denoise_burst_sigma_clip(
    frames: List[np.ndarray], sigma: float = 2.5, iters: int = 1
) -> np.ndarray:
    """
    Sigma clipping을 사용한 버스트 이미지 디노이징

    Args:
        frames: 입력 이미지 리스트
        sigma: 시그마 클리핑 임계값
        iters: 반복 횟수

    Returns:
        디노이징된 평균 이미지
    """
    arr = np.stack(frames, axis=0).astype(np.float32)  # (N,H,W[,C])
    m = arr.mean(axis=0, dtype=np.float32)
    s = arr.std(axis=0, dtype=np.float32) + 1e-10
    mask = np.abs(arr - m) <= sigma * s

    for _ in range(iters - 1):
        # 재계산
        safe = np.where(mask, arr, np.nan)
        m = np.nanmean(safe, axis=0).astype(np.float32)
        s = np.nanstd(safe, axis=0).astype(np.float32) + 1e-10
        mask = np.abs(arr - m) <= sigma * s

    # 최종 평균
    safe = np.where(mask, arr, np.nan)
    return np.nanmean(safe, axis=0).astype(np.float32)


def filter_by_brightness_outliers(
    images: List[np.ndarray],
    thresh: float = 3.5,  # 일반적으로 3.0~3.5가 보수적 임계값
    keep_edge_case: bool = True,  # MAD=0 등 특수 상황 처리
) -> Tuple[List[np.ndarray], List[int], List[int], Dict[str, Any]]:
    """
    밝기 기반 아웃라이어 필터링

    Args:
        images: 입력 이미지 리스트, 각 원소 shape=(H, W), dtype=float32
        thresh: 아웃라이어 임계값
        keep_edge_case: MAD=0 등 특수 상황 처리 여부

    Returns:
        filtered_images: 필터링된 이미지 리스트
        kept_indices: 유지된 인덱스 리스트
        dropped_indices: 제거된 인덱스 리스트
        stats: 통계 정보 딕셔너리
    """
    if len(images) == 0:
        return (
            images,
            [],
            [],
            {
                "means": np.array([]),
                "median": np.nan,
                "mad": np.nan,
            },
        )

    # (1) 프레임별 밝기 스칼라 요약(robustness 위해 nan-safe 사용)
    frame_means = np.array([np.nanmean(img) for img in images], dtype=np.float64)

    # (2) robust location/scale 추정치: median & MAD
    med = np.nanmedian(frame_means)
    mad = np.nanmedian(np.abs(frame_means - med))

    # (3) robust z-score (Leys et al., 2013 권장): 0.6745 * (x - median)/MAD
    if mad == 0 or not np.isfinite(mad):
        # 모든 프레임 밝기가 거의 동일한 경우 등
        if keep_edge_case:
            # 표준편차 기반으로 폴백(여전히 nan-safe)
            std = np.nanstd(frame_means)
            if std == 0 or not np.isfinite(std):
                # 완전히 동일하면 모두 유지
                mask_keep = np.ones_like(frame_means, dtype=bool)
            else:
                z = (frame_means - np.nanmean(frame_means)) / std
                mask_keep = np.abs(z) <= thresh
        else:
            mask_keep = np.ones_like(frame_means, dtype=bool)
    else:
        rzs = 0.6745 * (frame_means - med) / mad
        mask_keep = np.abs(rzs) <= thresh

    kept_indices = np.flatnonzero(mask_keep).tolist()
    dropped_indices = np.flatnonzero(~mask_keep).tolist()
    filtered_images = [images[i] for i in kept_indices]

    stats = {
        "means": frame_means,
        "median": med,
        "mad": mad,
        "threshold": thresh,
    }
    return filtered_images, kept_indices, dropped_indices, stats


def convert_lucid_images_to_float32(lucid_images: List[LucidImage]) -> List[np.ndarray]:
    """
    LucidImage 객체들을 float32 numpy 배열로 변환

    Args:
        lucid_images: LucidImage 객체 리스트

    Returns:
        변환된 float32 numpy 배열 리스트
    """
    images = []

    for lucid_img in lucid_images:
        if hasattr(lucid_img, "buffer_np") and lucid_img.buffer_np is not None:
            img_data = lucid_img.buffer_np

            # 24비트 HDR Bayer 처리 (H x W x 3 uint8 형식인 경우)
            if len(img_data.shape) == 3 and img_data.shape[2] == 3:
                # 3개의 uint8 채널을 24비트 정수로 결합
                combined = (
                    img_data[:, :, 0].astype(np.int32)
                    + (img_data[:, :, 1].astype(np.int32) << 8)
                    + (img_data[:, :, 2].astype(np.int32) << 16)
                )
                # 24비트 최대값으로 정규화
                float_img = combined.astype(np.float32) / (2**24 - 1)
            elif len(img_data.shape) == 3 and img_data.shape[2] == 2:
                # 12비트 이미지 처리
                combined = img_data[:, :, 0].astype(np.int32) + (
                    img_data[:, :, 1].astype(np.int32) << 8
                )
                float_img = combined.astype(np.float32) / (2**12 - 1)
            else:
                # 단일 채널 또는 이미 float 형식
                float_img = img_data.astype(np.float32)
                if float_img.max() > 1.0:
                    float_img = float_img / float_img.max()

            images.append(float_img)
        else:
            print(f"Warning: LucidImage has no valid buffer_np")

    return images


def process_high_gain_frames(
    lucid_images: List[LucidImage], progress_callback=None
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """
    High-gain LucidImage 프레임들을 처리하여 단일 합성 이미지 생성

    Args:
        lucid_images: LucidImage 객체 리스트
        progress_callback: 진행상황 콜백 함수 (선택사항)

    Returns:
        merged_image: 합성된 이미지 (float32)
        processing_stats: 처리 통계 정보
    """
    if not lucid_images or len(lucid_images) == 0:
        raise ValueError("No LucidImage frames provided")

    if progress_callback:
        progress_callback("extracting_images", 10)

    # LucidImage에서 numpy 배열 추출 및 변환
    images = convert_lucid_images_to_float32(lucid_images)

    if len(images) == 0:
        raise ValueError("No valid image data found in LucidImage objects")

    if progress_callback:
        progress_callback("format_conversion", 30)

    # 단일 이미지인 경우 바로 반환
    if len(images) == 1:
        processing_stats = {
            "total_frames": 1,
            "kept_frames": 1,
            "dropped_frames": 0,
            "filtering_applied": False,
        }
        return images[0], processing_stats

    if progress_callback:
        progress_callback("outlier_filtering", 50)

    # 아웃라이어 필터링
    filtered_images, kept_indices, dropped_indices, filter_stats = (
        filter_by_brightness_outliers(images, thresh=3.5, keep_edge_case=True)
    )

    print(
        f"Filtered {len(filtered_images)} frames, dropped {len(dropped_indices)} outliers"
    )

    if progress_callback:
        progress_callback("denoising", 70)

    # 시그마 클리핑 디노이징
    if len(filtered_images) > 1:
        merged_image = denoise_burst_sigma_clip(filtered_images, sigma=2.5, iters=2)
    else:
        merged_image = filtered_images[0] if filtered_images else images[0]

    if progress_callback:
        progress_callback("finalizing", 90)

    processing_stats = {
        "total_frames": len(images),
        "kept_frames": len(kept_indices),
        "dropped_frames": len(dropped_indices),
        "filtering_applied": True,
    }

    # 필터 통계를 별도로 추가
    processing_stats.update(filter_stats)

    if progress_callback:
        progress_callback("completed", 100)

    return merged_image, processing_stats
