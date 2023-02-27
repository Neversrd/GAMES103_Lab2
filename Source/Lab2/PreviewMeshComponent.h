// Copyright (c) 2022, Li Huang. All rights reserved.
#pragma once

#include "CoreMinimal.h"
#include "Components/MeshComponent.h"
#include "PreviewMeshComponent.generated.h"

/**
 * 
 */

class FPrimitiveSceneProxy;

UENUM(BlueprintType)
enum class EMeshDataState : uint8 {
  Uninitialized,
  UpToDate,
  PositionDirty,
  AllDirty
};

UCLASS(meta = (BlueprintSpawnableComponent), ClassGroup = Custom)
class LAB2_API UPreviewMeshComponent : public UMeshComponent {
  GENERATED_BODY()

 public:
  UPreviewMeshComponent();

  TArray<FVector> positions_;
  TArray<FVector2D> uvs_;
  TArray<uint32> indices_;
  TArray<FVector> normals_;
  TArray<FVector> tangents_;
  TArray<FVector> bi_tangents_;
  uint32 num_vertices_;
  uint32 num_indices_;

  void SetMeshData(const TArray<FVector>& positions,
                   const TArray<uint32>& indices,
                   const TArray<FVector>& normals,
                   const TArray<FVector>& tangents,
                   const TArray<FVector>& bi_tangents,
                   const TArray<FVector2D>& uvs);

  void UpdateMeshVertices(const TArray<FVector>& positions);

  void ClearMeshData();

  UPROPERTY(EditAnywhere);
  UStaticMesh* static_mesh_;

  // Bounds
  UPROPERTY()
  FVector box_min_;
  UPROPERTY()
  FVector box_max_;

  uint32 MeshUpdateCounter;

  EMeshDataState mesh_data_state_ = EMeshDataState::Uninitialized;

 private:
  FPrimitiveSceneProxy* CreateSceneProxy() override;
  int32 GetNumMaterials() const override;
  FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
  void TickComponent(float DeltaTime, enum ELevelTick TickType,
                     FActorComponentTickFunction* ThisTickFunction) override;
  void SendRenderDynamicData_Concurrent() override;
  void CreateRenderState_Concurrent(
      FRegisterComponentContext* Context) override;

  void UpdateBoundingBox();

  void PostEditChangeProperty(
      FPropertyChangedEvent& PropertyChangedEvent) override;

  friend class FPreviewMeshSceneProxy;
};
