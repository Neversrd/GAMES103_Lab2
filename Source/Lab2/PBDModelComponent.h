// Copyright (c) 2022, Li Huang. All rights reserved.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PreviewMeshComponent.h"
#include "PBDModelComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class LAB2_API UPBDModelComponent : public UActorComponent
{
  GENERATED_BODY()

public:	
  // Sets default values for this component's properties
  UPBDModelComponent();

protected:
  // Called when the game starts
  virtual void BeginPlay() override;

public:	
  // Called every frame
  virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

  void Collision_Handling();
  void StrainLimiting();
  
  // t
  UPROPERTY(EditAnywhere)
  float delta_t_ = 0.0333f;
  UPROPERTY(EditAnywhere)
  float damping_ = 0.99f;
  UPROPERTY(EditAnywhere)
  float alpha_ = 0.2f;

  UPROPERTY(EditAnywhere)
  FVector gravity_ = FVector(0.f, 0.f, -981.f);

  UPROPERTY(EditAnywhere)
  AActor* sphere_actor_;
  
  // E
  TArray<FVector2D> unique_edges_;
  // L
  TArray<float> unique_edge_lengths_;
  // V
  TArray<FVector> velocities_;

  UPreviewMeshComponent* preview_mesh_component_;
};
