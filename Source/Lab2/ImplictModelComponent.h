// Copyright (c) 2022, Li Huang. All rights reserved.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PreviewMeshComponent.h"
#include "ImplictModelComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class LAB2_API UImplictModelComponent : public UActorComponent
{
  GENERATED_BODY()

public:	
  // Sets default values for this component's properties
  UImplictModelComponent();

protected:
  // Called when the game starts
  virtual void BeginPlay() override;

public:	
  // Called every frame
  virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

  void Collision_Handling();
  void Get_Gradient(const TArray<FVector>& X, const TArray<FVector>& X_hat,
                    float dt, TArray<FVector>& G);

  
  // t
  UPROPERTY(EditAnywhere)
  float delta_t_ = 0.0333f;
  UPROPERTY(EditAnywhere)
  float mass_ = 1.f;
  UPROPERTY(EditAnywhere)
  float damping_ = 0.99f;
  UPROPERTY(EditAnywhere)
  float rho_ = 0.995f;
  UPROPERTY(EditAnywhere)
  float spring_k_ = 800000.f;

  UPROPERTY(EditAnywhere)
  FVector gravity_ = FVector(0.f, 0.f, -981.f);

  UPROPERTY(EditAnywhere)
  AActor* sphere_actor_;

  UPROPERTY(EditAnywhere)
  bool use_chebyshev_acceleration_ = false;
  
  // E
  TArray<FVector2D> unique_edges_;
  // L
  TArray<float> unique_edge_lengths_;
  // V
  TArray<FVector> velocities_;

  UPreviewMeshComponent* preview_mesh_component_;
};
