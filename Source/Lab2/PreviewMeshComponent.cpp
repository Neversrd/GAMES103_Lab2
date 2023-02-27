// Copyright (c) 2022, Li Huang. All rights reserved.
#include "PreviewMeshComponent.h"

class FPreviewMeshIndexBuffer : public FIndexBuffer {
 public:
  int32 NumIndices;
  void InitRHI() override {
    FRHIResourceCreateInfo CreateInfo;
    IndexBufferRHI = RHICreateIndexBuffer(
        sizeof(int32), NumIndices * sizeof(int32), BUF_Dynamic, CreateInfo);
  }
};

class FPreviewMeshSceneProxy : public FPrimitiveSceneProxy {
 public:
  FPreviewMeshSceneProxy(UPreviewMeshComponent* Component)
      : FPrimitiveSceneProxy(Component),
        ParentComponent(Component),
        MaterialRelevance(
            Component->GetMaterialRelevance(GetScene().GetFeatureLevel())),
        VertexFactory(GetScene().GetFeatureLevel(),
                      "FPreviewMeshSceneProxy") {
    vertex_buffers_allocated_num_vertices_ = 0;
    index_buffer_allocated_num_indices_ = 0;
    NonSimulatedMeshCounter = 0;
    IndexBuffer.NumIndices = index_buffer_allocated_num_indices_;

    VertexBuffers.InitWithDummyData(&VertexFactory, 16);
    IndexBuffer.NumIndices = 16;
    BeginInitResource(&IndexBuffer);

    Material = Component->GetMaterial(0);
    if (!Material) {
      Material = UMaterial::GetDefaultMaterial(MD_Surface);
    }
  }

  size_t GetTypeHash() const override {
    static size_t UniquePointer;
    return reinterpret_cast<size_t>(&UniquePointer);
  }

  virtual ~FPreviewMeshSceneProxy() {
    VertexBuffers.PositionVertexBuffer.ReleaseResource();
    VertexBuffers.StaticMeshVertexBuffer.ReleaseResource();
    VertexBuffers.ColorVertexBuffer.ReleaseResource();
    IndexBuffer.ReleaseResource();
    VertexFactory.ReleaseResource();
  }

  void GetDynamicMeshElements(const TArray<const FSceneView*>& Views,
                              const FSceneViewFamily& ViewFamily,
                              uint32 VisibilityMap,
                              FMeshElementCollector& Collector) const override {
    QUICK_SCOPE_CYCLE_COUNTER(
        STAT_FPreviewMeshSceneProxy_GetDynamicMeshElements);
    if (!vertex_buffers_allocated_num_vertices_ ||
        !index_buffer_allocated_num_indices_) {
      return;
    }

    const bool bWireFrame =
        AllowDebugViewmodes() && ViewFamily.EngineShowFlags.Wireframe;
    auto WireframeMaterialInstance = new FColoredMaterialRenderProxy(
        GEngine->WireframeMaterial
            ? GEngine->WireframeMaterial->GetRenderProxy()
            : NULL,
        FLinearColor(0, 0.5f, 1.f));
    Collector.RegisterOneFrameMaterialProxy(WireframeMaterialInstance);
    FMaterialRenderProxy* MaterialProxy = NULL;
    if (bWireFrame) {
      MaterialProxy = WireframeMaterialInstance;
    } else {
      MaterialProxy = Material->GetRenderProxy();
    }
    for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++) {
      if (VisibilityMap & (1 << ViewIndex)) {
        const FSceneView* View = Views[ViewIndex];
        FMeshBatch& Mesh = Collector.AllocateMesh();
        FMeshBatchElement& BatchElement = Mesh.Elements[0];
        BatchElement.IndexBuffer = &IndexBuffer;
        Mesh.bWireframe = bWireFrame;
        Mesh.VertexFactory = &VertexFactory;
        Mesh.MaterialRenderProxy = MaterialProxy;
        bool bHasPrecomputedVolumetricLightmap;
        FMatrix PreviousLocalToWorld;
        int32 SingleCaptureIndex;
        bool bOutputVelocity;
        GetScene().GetPrimitiveUniformShaderParameters_RenderThread(
            GetPrimitiveSceneInfo(), bHasPrecomputedVolumetricLightmap,
            PreviousLocalToWorld, SingleCaptureIndex, bOutputVelocity);
        FDynamicPrimitiveUniformBuffer& DynamicPrimitiveUniformBuffer =
            Collector
                .AllocateOneFrameResource<FDynamicPrimitiveUniformBuffer>();
        DynamicPrimitiveUniformBuffer.Set(
            GetLocalToWorld(), PreviousLocalToWorld, GetBounds(),
            GetLocalBounds(), true, bHasPrecomputedVolumetricLightmap,
            DrawsVelocity(), bOutputVelocity);
        BatchElement.PrimitiveUniformBufferResource =
            &DynamicPrimitiveUniformBuffer.UniformBuffer;
        BatchElement.FirstIndex = 0;
        BatchElement.NumPrimitives = index_buffer_allocated_num_indices_ / 3;
        BatchElement.MinVertexIndex = 0;
        BatchElement.MaxVertexIndex = vertex_buffers_allocated_num_vertices_;
        Mesh.ReverseCulling = IsLocalToWorldDeterminantNegative();
        Mesh.Type = PT_TriangleList;
        Mesh.DepthPriorityGroup = SDPG_World;
        Mesh.bCanApplyViewModeOverrides = false;
        Collector.AddMesh(ViewIndex, Mesh);
      }
    }
  }

  virtual FPrimitiveViewRelevance GetViewRelevance(
      const FSceneView* View) const override {
    FPrimitiveViewRelevance Result;
    Result.bDrawRelevance = IsShown(View);
    Result.bShadowRelevance = IsShadowCast(View);
    Result.bDynamicRelevance = true;
    Result.bRenderInMainPass = ShouldRenderInMainPass();
    Result.bUsesLightingChannels =
        GetLightingChannelMask() != GetDefaultLightingChannelMask();
    Result.bRenderCustomDepth = ShouldRenderCustomDepth();
    Result.bTranslucentSelfShadow = bCastVolumetricTranslucentShadow;
    MaterialRelevance.SetPrimitiveViewRelevance(Result);
    Result.bVelocityRelevance =
        IsMovable() && Result.bOpaque && Result.bRenderInMainPass;
    return Result;
  }

  virtual bool CanBeOccluded() const override {
    return !MaterialRelevance.bDisableDepthTest;
  }

  uint32 GetMemoryFootprint(void) const override {
    return (sizeof(*this) + GetAllocatedSize());
  }

  void BuildMeshData_RenderThread() {
    check(IsInRenderingThread());

    BuildMesh();
  }

  void BuildMesh() {
    if (!ParentComponent) {
      return;
    }
    if (ParentComponent->num_vertices_ <= 0 ||
        ParentComponent->num_indices_ <= 0) {
      return;
    }
    if (ParentComponent->mesh_data_state_ == EMeshDataState::UpToDate) {
      return;
    }
    if (ParentComponent->mesh_data_state_ == EMeshDataState::PositionDirty) {
      UpdateVertices();
      return;
    }
    ParentComponent->mesh_data_state_ = EMeshDataState::UpToDate;
    // NonSimulatedMeshCounter = ParentComponent->MeshUpdateCounter;
    if (ParentComponent->num_vertices_ >
        vertex_buffers_allocated_num_vertices_) {
      VertexBuffers.PositionVertexBuffer.ReleaseResource();
      VertexBuffers.StaticMeshVertexBuffer.ReleaseResource();
      VertexBuffers.ColorVertexBuffer.ReleaseResource();
      VertexBuffers.InitWithDummyData(&VertexFactory,
                                      ParentComponent->num_vertices_);
      vertex_buffers_allocated_num_vertices_ =
          static_cast<uint32>(ParentComponent->num_vertices_);
    }

    if (ParentComponent->num_indices_ > index_buffer_allocated_num_indices_) {
      IndexBuffer.ReleaseResource();
      IndexBuffer.NumIndices = ParentComponent->num_indices_;
      BeginInitResource(&IndexBuffer);

      index_buffer_allocated_num_indices_ = ParentComponent->num_indices_;
    }

    // Set mesh vertex data on CPU
    for (uint32 i = 0; i < ParentComponent->num_vertices_; i++) {
      VertexBuffers.PositionVertexBuffer.VertexPosition(i) =
          ParentComponent->positions_[i];

      VertexBuffers.StaticMeshVertexBuffer.SetVertexTangents(
          i, ParentComponent->tangents_[i], ParentComponent->bi_tangents_[i],
          ParentComponent->normals_[i]);

      VertexBuffers.StaticMeshVertexBuffer.SetVertexUV(
          i, 0, ParentComponent->uvs_[i]);

      VertexBuffers.ColorVertexBuffer.VertexColor(i) = FColor(255, 255, 255);
    }

    // Copy mesh vertex data from CPU to GPU
    {
      auto& position_buffer = VertexBuffers.PositionVertexBuffer;
      // Get GPU pointer of position_buffer
      void* VertexBufferData = RHILockVertexBuffer(
          position_buffer.VertexBufferRHI, 0,
          position_buffer.GetNumVertices() * position_buffer.GetStride(),
          RLM_WriteOnly);
      // Copy data from CPU to GPU
      FMemory::Memcpy(
          VertexBufferData, position_buffer.GetVertexData(),
          position_buffer.GetNumVertices() * position_buffer.GetStride());
      RHIUnlockVertexBuffer(position_buffer.VertexBufferRHI);
    }

    // normal, tangent and bitangent
    {
      auto& mesh_buffer = VertexBuffers.StaticMeshVertexBuffer;
      void* VertexBufferData =
          RHILockVertexBuffer(mesh_buffer.TangentsVertexBuffer.VertexBufferRHI,
                              0, mesh_buffer.GetTangentSize(), RLM_WriteOnly);
      FMemory::Memcpy(VertexBufferData, mesh_buffer.GetTangentData(),
                      mesh_buffer.GetTangentSize());
      RHIUnlockVertexBuffer(mesh_buffer.TangentsVertexBuffer.VertexBufferRHI);
    }

    // uv
    {
      auto& mesh_buffer = VertexBuffers.StaticMeshVertexBuffer;
      void* VertexBufferData =
          RHILockVertexBuffer(mesh_buffer.TexCoordVertexBuffer.VertexBufferRHI,
                              0, mesh_buffer.GetTexCoordSize(), RLM_WriteOnly);
      FMemory::Memcpy(VertexBufferData, mesh_buffer.GetTexCoordData(),
                      mesh_buffer.GetTexCoordSize());
      RHIUnlockVertexBuffer(mesh_buffer.TexCoordVertexBuffer.VertexBufferRHI);
    }

    // indices
    {
      void* IndexBufferData = RHILockIndexBuffer(
          IndexBuffer.IndexBufferRHI, 0,
          ParentComponent->num_indices_ * sizeof(uint32), RLM_WriteOnly);
      FMemory::Memcpy(IndexBufferData, ParentComponent->indices_.GetData(),
                      ParentComponent->num_indices_ * sizeof(uint32));
      RHIUnlockIndexBuffer(IndexBuffer.IndexBufferRHI);
    }
  }

  void UpdatePositionsData_RenderThread() {
    check(IsInRenderingThread());

    UpdateVertices();
  }

  void UpdateVertices() {
    if (!ParentComponent ||
        ParentComponent->mesh_data_state_ != EMeshDataState::PositionDirty ||
        ParentComponent->num_vertices_ <= 0 ||
        ParentComponent->num_vertices_ !=
            vertex_buffers_allocated_num_vertices_) {
      return;
    }
    ParentComponent->mesh_data_state_ = EMeshDataState::UpToDate;
    
    // Set mesh vertex position data on CPU
    for (uint32 i = 0; i < ParentComponent->num_vertices_; i++) {
      VertexBuffers.PositionVertexBuffer.VertexPosition(i) =
          ParentComponent->positions_[i];
    }

    // Copy mesh vertex data from CPU to GPU
    {
      auto& position_buffer = VertexBuffers.PositionVertexBuffer;
      // Get GPU pointer of position_buffer
      void* VertexBufferData = RHILockVertexBuffer(
          position_buffer.VertexBufferRHI, 0,
          position_buffer.GetNumVertices() * position_buffer.GetStride(),
          RLM_WriteOnly);
      // Copy data from CPU to GPU
      FMemory::Memcpy(
          VertexBufferData, position_buffer.GetVertexData(),
          position_buffer.GetNumVertices() * position_buffer.GetStride());
      RHIUnlockVertexBuffer(position_buffer.VertexBufferRHI);
    }
  }

 private:
  UMaterialInterface* Material;
  FStaticMeshVertexBuffers VertexBuffers;
  FPreviewMeshIndexBuffer IndexBuffer;
  FLocalVertexFactory VertexFactory;

  FMaterialRelevance MaterialRelevance;
  UPreviewMeshComponent* ParentComponent;

  uint32 vertex_buffers_allocated_num_vertices_ = 0;
  uint32 index_buffer_allocated_num_indices_ = 0;

  uint32 NonSimulatedMeshCounter;
};

UPreviewMeshComponent::UPreviewMeshComponent() {
  PrimaryComponentTick.bCanEverTick = true;
  bTickInEditor = true;
  bAutoActivate = true;

  // MeshUpdateCounter = 0;
  mesh_data_state_ = EMeshDataState::Uninitialized;
}

void UPreviewMeshComponent::PostEditChangeProperty(
    FPropertyChangedEvent& PropertyChangedEvent) {
  Super::PostEditChangeProperty(PropertyChangedEvent);

  if (PropertyChangedEvent.Property) {
    if (PropertyChangedEvent.Property->GetFName() ==
        GET_MEMBER_NAME_CHECKED(UPreviewMeshComponent, static_mesh_)) {
      if (static_mesh_) {
        const FStaticMeshVertexBuffers& vertex_buffers =
            static_mesh_->RenderData->LODResources[0].VertexBuffers;
        // steal initial mesh data from static_mesh_
        TArray<FVector> pos;
        TArray<FVector2D> uv;
        TArray<FVector> norms;
        TArray<FVector> tangents;
        TArray<FVector> bitangents;
        size_t num_vertices =
            vertex_buffers.PositionVertexBuffer.GetNumVertices();
        pos.SetNum(num_vertices);
        uv.SetNum(num_vertices);
        norms.SetNum(num_vertices);
        tangents.SetNum(num_vertices);
        bitangents.SetNum(num_vertices);
        for (size_t i = 0; i < num_vertices; i++) {
          pos[i] = vertex_buffers.PositionVertexBuffer.VertexPosition(i);
          uv[i] = vertex_buffers.StaticMeshVertexBuffer.GetVertexUV(i, 0);
          tangents[i] = vertex_buffers.StaticMeshVertexBuffer.VertexTangentX(i);
          bitangents[i] =
              vertex_buffers.StaticMeshVertexBuffer.VertexTangentY(i);
          norms[i] = vertex_buffers.StaticMeshVertexBuffer.VertexTangentZ(i);
        }
        TArray<uint32> indices;
        static_mesh_->RenderData->LODResources[0].IndexBuffer.GetCopy(indices);
        SetMeshData(pos, indices, norms, tangents, bitangents, uv);
      }
    }
  }
}

int32 UPreviewMeshComponent::GetNumMaterials() const { return 1; }

void UPreviewMeshComponent::TickComponent(
    float DeltaTime, enum ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction) {
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
  UpdateComponentToWorld();
}

void UPreviewMeshComponent::CreateRenderState_Concurrent(
    FRegisterComponentContext* Context) {
  Super::CreateRenderState_Concurrent(Context);
  SendRenderDynamicData_Concurrent();
}

void UPreviewMeshComponent::SendRenderDynamicData_Concurrent() {
  FPreviewMeshSceneProxy* MeshSceneProxy =
      reinterpret_cast<FPreviewMeshSceneProxy*>(SceneProxy);
  if (MeshSceneProxy) {
    ENQUEUE_RENDER_COMMAND(FSendPreviewMeshComponentDynamicData)
    ([this, MeshSceneProxy](FRHICommandListImmediate& RHICmdList) {
      if (mesh_data_state_ == EMeshDataState::Uninitialized ||
          mesh_data_state_ == EMeshDataState::AllDirty) {
        MeshSceneProxy->BuildMeshData_RenderThread();
      } else if (mesh_data_state_ == EMeshDataState::PositionDirty) {
        MeshSceneProxy->UpdatePositionsData_RenderThread();
      }
    });
  }
}

FPrimitiveSceneProxy* UPreviewMeshComponent::CreateSceneProxy() {
  return new FPreviewMeshSceneProxy(this);
}

FBoxSphereBounds UPreviewMeshComponent::CalcBounds(
    const FTransform& LocalToWorld) const {
  FBoxSphereBounds NewBounds(ForceInit);
  NewBounds.Origin = (box_min_ + box_max_) / 2;
  NewBounds.BoxExtent = (box_max_ - box_min_) / 2;
  NewBounds.SphereRadius = NewBounds.BoxExtent.Size();
  return NewBounds.TransformBy(LocalToWorld);
}

void UPreviewMeshComponent::SetMeshData(
    const TArray<FVector>& positions, const TArray<uint32>& indices,
    const TArray<FVector>& normals, const TArray<FVector>& tangents,
    const TArray<FVector>& bi_tangents, const TArray<FVector2D>& uvs) {
  if (positions.Num() != normals.Num() || positions.Num() != tangents.Num() ||
      positions.Num() != bi_tangents.Num() || positions.Num() != uvs.Num()) {
    return;
  }
  positions_ = positions;
  uvs_ = uvs;
  indices_ = indices;
  normals_ = normals;
  tangents_ = tangents;
  bi_tangents_ = bi_tangents;

  num_vertices_ = positions.Num();
  num_indices_ = indices.Num();

  mesh_data_state_ = EMeshDataState::AllDirty;

  MarkRenderDynamicDataDirty();
}

void UPreviewMeshComponent::UpdateMeshVertices(const TArray<FVector>& positions) {
  if (positions.Num() != normals_.Num() || positions.Num() != tangents_.Num() ||
      positions.Num() != bi_tangents_.Num() || positions.Num() != uvs_.Num()) {
    return;
  }
  positions_ = positions;
  num_vertices_ = positions.Num();
  UpdateBoundingBox();

  //mesh_data_state_ = EMeshDataState::PositionDirty;
  mesh_data_state_ = EMeshDataState::AllDirty;

  MarkRenderDynamicDataDirty();
}

void UPreviewMeshComponent::ClearMeshData() {
  positions_.Empty();
  uvs_.Empty();
  normals_.Empty();
  tangents_.Empty();
  bi_tangents_.Empty();
  indices_.Empty();
  num_vertices_ = 0;
  num_indices_ = 0;

  mesh_data_state_ = EMeshDataState::Uninitialized;
}

void UPreviewMeshComponent::UpdateBoundingBox() {
  box_min_ = FVector(HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
  box_max_ = -FVector(HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
  for (auto pos : positions_) {
    box_min_ = box_min_.ComponentMin(pos);
    box_max_ = box_max_.ComponentMax(pos);
  }
}