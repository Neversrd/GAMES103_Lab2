// Copyright (c) 2022, Li Huang. All rights reserved.
#include "PBDModelComponent.h"
#include "MathUtil.h"

// Sets default values for this component's properties
UPBDModelComponent::UPBDModelComponent()
{
  // Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
  // off to improve performance if you don't need them.
  PrimaryComponentTick.bCanEverTick = true;

  // ...
}


// Called when the game starts
void UPBDModelComponent::BeginPlay() {
  Super::BeginPlay();

  // ...
  preview_mesh_component_ =
      GetOwner()->FindComponentByClass<UPreviewMeshComponent>();

  if (!preview_mesh_component_) {
    return;
  }

  int n = 21;
  TArray<FVector> pos;
  TArray<FVector2D> uv;
  TArray<FVector> norms;
  TArray<FVector> tangents;
  TArray<FVector> bitangents;
  pos.SetNum(n * n);
  uv.SetNum(n * n);
  norms.SetNum(n * n);
  tangents.SetNum(n * n);
  bitangents.SetNum(n * n);

  TArray<uint32> triangles;
  triangles.SetNum((n - 1) * (n - 1) * 6);

  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < n; j++) {
      pos[i * n + j] =
          FVector(50.f - 100.f * j / (n - 1), 50.f - 100.f * i / (n - 1), 0.f);
      uv[i * n + j] = FVector2D(j / (n - 1.0f), i / (n - 1.0f));
      norms[i * n + j] = FVector(0.f, 0.f, -1.f);
      tangents[i * n + j] = FVector(1.f, 0.f, 0.f);
      bitangents[i * n + j] = FVector(0.f, -1.f, 0.f);
    }
  }
  int counter = 0;
  for (size_t i = 0; i < n - 1; i++) {
    for (size_t j = 0; j < n - 1; j++) {
      triangles[counter * 6 + 0] = i * n + j;
      triangles[counter * 6 + 1] = i * n + j + 1;
      triangles[counter * 6 + 2] = (i + 1) * n + j + 1;
      triangles[counter * 6 + 3] = i * n + j;
      triangles[counter * 6 + 4] = (i + 1) * n + j + 1;
      triangles[counter * 6 + 5] = (i + 1) * n + j;
      counter++;
    }
  }

  preview_mesh_component_->SetMeshData(pos, triangles, norms, tangents,
                                       bitangents, uv);

  TArray<FVector2D> original_edges;
  original_edges.SetNum(triangles.Num());
  for (int i = 0; i < triangles.Num(); i += 3) {
    original_edges[i].X = triangles[i + 0];
    original_edges[i].Y = triangles[i + 1];
    if (original_edges[i].X >= original_edges[i].Y) {
      Swap(original_edges[i].X, original_edges[i].Y);
    }

    original_edges[i + 1].X = triangles[i + 1];
    original_edges[i + 1].Y = triangles[i + 2];
    if (original_edges[i + 1].X >= original_edges[i + 1].Y) {
      Swap(original_edges[i + 1].X, original_edges[i + 1].Y);
    }

    original_edges[i + 2].X = triangles[i + 2];
    original_edges[i + 2].Y = triangles[i + 0];
    if (original_edges[i + 2].X >= original_edges[i + 2].Y) {
      Swap(original_edges[i + 2].X, original_edges[i + 2].Y);
    }
  }

  original_edges.Sort([](const FVector2D& a, const FVector2D& b) {
    if (a.X != b.X) {
      return a.X < b.X;
    }
    return a.Y < b.Y;
  });

  int unique_edge_num = 0;
  for (int i = 0; i < original_edges.Num(); i++) {
    if (i == 0 || original_edges[i].X != original_edges[i - 1].X ||
        original_edges[i].Y != original_edges[i - 1].Y) {
      unique_edge_num++;
    }
  }

  unique_edges_.SetNum(unique_edge_num);
  unique_edge_lengths_.SetNum(unique_edge_num);
  for (int i = 0, e = 0; i < original_edges.Num(); i++) {
    if (i == 0 || original_edges[i].X != original_edges[i - 1].X ||
        original_edges[i].Y != original_edges[i - 1].Y) {
      unique_edges_[e] = original_edges[i];
      unique_edge_lengths_[e] =
          FVector(pos[unique_edges_[e].X] - pos[unique_edges_[e].Y]).Size();
      e++;
    }
  }

  velocities_.SetNumZeroed(pos.Num());
}

void UPBDModelComponent::Collision_Handling() {
  if (!sphere_actor_) {
    return;
  }
  
  //Mesh mesh = GetComponent<MeshFilter>().mesh;
  //Vector3[] X = mesh.vertices;

  TArray<FVector> X = preview_mesh_component_->positions_;
  FVector sphere_center = sphere_actor_->GetActorLocation();
  float sphere_radius = 25.f;
  float one_over_dt = 1.f / delta_t_;
  // Handle colllision.
  for (size_t i = 0; i < X.Num(); i++) {
    // First, check if this vertex is inside the sdf we are testing
    float phi = ComputePhiSphere(X[i], sphere_center, sphere_radius);
    if (phi >= 0.f) {
      continue;
    }
    // Second, check if this vertex is escaping from the sdf
    const FVector& velocity = velocities_[i];
    FVector N = X[i] - sphere_center;
    N.Normalize();

    if (FVector::DotProduct(velocity, N) >= 0.f) {
      continue;
    }

    // Collision is detected
    velocities_[i] += one_over_dt * (sphere_center + sphere_radius * N - X[i]);
    X[i] = sphere_center + sphere_radius * N;
  }

  //mesh.vertices = X;
  preview_mesh_component_->UpdateMeshVertices(X);
}

void UPBDModelComponent::StrainLimiting() {
  //Mesh mesh = GetComponent<MeshFilter>().mesh;
  //Vector3[] vertices = mesh.vertices;
  TArray<FVector> X = preview_mesh_component_->positions_;

  // Apply PBD here.
  TArray<FVector> sum_x;
  sum_x.SetNumZeroed(X.Num());
  TArray<uint32> sum_n;
  sum_n.SetNumZeroed(X.Num());
  for (size_t e = 0; e < unique_edges_.Num(); e++) {
    size_t i = unique_edges_[e].X;
    size_t j = unique_edges_[e].Y;
    FVector xi_xj = X[i] - X[j];
    xi_xj.Normalize();
    sum_x[i] += 0.5f * (X[i] + X[j] + unique_edge_lengths_[e] * xi_xj);
    sum_n[i]++;
    sum_x[j] += 0.5f * (X[i] + X[j] - unique_edge_lengths_[e] * xi_xj);
    sum_n[j]++;
  }

  float one_over_dt = 1.f / delta_t_;

  for (size_t i = 0; i < X.Num(); i++) {
    if (i == 0 || i == 20) {
      continue;
    }
    velocities_[i] +=
        one_over_dt * ((alpha_ * X[i] + sum_x[i]) / (alpha_ + sum_n[i]) - X[i]);
    X[i] = (alpha_ * X[i] + sum_x[i]) / (alpha_ + sum_n[i]);
  }

  //mesh.vertices = vertices;
  preview_mesh_component_->UpdateMeshVertices(X);
}

// Called every frame
void UPBDModelComponent::TickComponent(
    float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction) {
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

  if (!preview_mesh_component_ || !preview_mesh_component_->positions_.Num()) {
    return;
  }

  TArray<FVector> X = preview_mesh_component_->positions_;
  
  for (int i = 0; i < X.Num(); i++) {
    if (i == 0 || i == 20) {
      continue;
    }
    // Initial Setup
    velocities_[i] *= damping_;
    velocities_[i] += gravity_ * delta_t_;
    X[i] += velocities_[i] * delta_t_;
  }
  // mesh.vertices = X;
  preview_mesh_component_->UpdateMeshVertices(X);

  for (int l = 0; l < 32; l++) {
    StrainLimiting();
  }

  Collision_Handling();

  // No need to recalculate normals in UE
  // mesh.RecalculateNormals();
}

