// Copyright (c) 2022, Li Huang. All rights reserved.
#include "ImplictModelComponent.h"
#include "MathUtil.h"

// Sets default values for this component's properties
UImplictModelComponent::UImplictModelComponent()
{
  // Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
  // off to improve performance if you don't need them.
  PrimaryComponentTick.bCanEverTick = true;

  // ...
}


// Called when the game starts
void UImplictModelComponent::BeginPlay() {
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

void UImplictModelComponent::Collision_Handling() {
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

    velocities_[i] += one_over_dt * (sphere_center + sphere_radius * N - X[i]);
    X[i] = sphere_center + sphere_radius * N;
  }

  //mesh.vertices = X;
  preview_mesh_component_->UpdateMeshVertices(X);
}

void UImplictModelComponent::Get_Gradient(const TArray<FVector>& X,
                                          const TArray<FVector>& X_hat, float dt,
                                          TArray<FVector>& G) {
  // Momentum and Gravity.
  float one_over_dt2 = 1.f / (dt * dt);
  for (size_t i = 0; i < X.Num(); i++) {
    G[i] = one_over_dt2 * mass_ * (X[i] - X_hat[i]); 
    G[i] -= gravity_;
  }
  
  // Spring Force.
  for (size_t i = 0; i < unique_edges_.Num(); i++) {
    FVector xi_xj = X[unique_edges_[i].X] - X[unique_edges_[i].Y];
    FVector minus_force =
        spring_k_ * (1 - unique_edge_lengths_[i] / xi_xj.Size()) * xi_xj;
    G[unique_edges_[i].X] += minus_force;
    G[unique_edges_[i].Y] -= minus_force;
  }
}

// Called every frame
void UImplictModelComponent::TickComponent(
    float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction) {
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

  if (!preview_mesh_component_ || !preview_mesh_component_->positions_.Num()) {
    return;
  }

  TArray<FVector> X = preview_mesh_component_->positions_;
  TArray<FVector> last_X;
  last_X.SetNumZeroed(X.Num());
  TArray<FVector> X_hat;
  X_hat.SetNum(X.Num());
  TArray<FVector> G;
  G.SetNum(X.Num());

  // Initial Setup.
  for (size_t i = 0; i < velocities_.Num(); i++) {
    velocities_[i] *= damping_;
    X_hat[i] = X[i] + velocities_[i] * delta_t_;
  }
  X = X_hat;

  float one_over_dt2 = 1.f / (delta_t_ * delta_t_);
  if (!use_chebyshev_acceleration_) {
    for (int k = 0; k < 32; k++) {
      Get_Gradient(X, X_hat, delta_t_, G);

      // Update X by gradient.
      for (size_t i = 0; i < X.Num(); i++) {
        X[i] -= G[i] / (one_over_dt2 * mass_ + 4 * spring_k_);
      }
    }
  } else {
    float omega = 1.f;
    for (int k = 0; k < 32; k++) {
      Get_Gradient(X, X_hat, delta_t_, G);
      if (k == 1) {
        omega = 2.f / (2.f - rho_ * rho_);
      } else if (k > 1) {
        omega = 4.f / (4.f - rho_ * rho_ * omega);
      }

      // Update X by gradient.
      for (size_t i = 0; i < X.Num(); i++) {
        FVector old_X = X[i];
        X[i] -= G[i] / (one_over_dt2 * mass_ + 4 * spring_k_);
        X[i] = omega * X[i] + (1 - omega) * last_X[i];
        last_X[i] = old_X;
      }
    }
  }

  // Finishing.
  for (size_t i = 0; i < X.Num(); i++) {
    velocities_[i] += (X[i] - X_hat[i]);
    
    // fix fixed particles
    if (i == 0 || i == 20) {
      X[i] = preview_mesh_component_->positions_[i];
      velocities_[i] = FVector::ZeroVector;
    }
  }

  //mesh.vertices = X;
  preview_mesh_component_->UpdateMeshVertices(X);
  Collision_Handling();
}

