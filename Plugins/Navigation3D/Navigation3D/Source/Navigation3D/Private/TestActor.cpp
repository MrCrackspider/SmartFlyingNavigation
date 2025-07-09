// Fill out your copyright notice in the Description page of Project Settings.


#include "TestActor.h"

#include "Components/SplineComponent.h"

ATestActor::ATestActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PathSpline = CreateDefaultSubobject<USplineComponent>(TEXT("PathSpline"));
	RootComponent = PathSpline;
}

// Called when the game starts or when spawned
void ATestActor::BeginPlay()
{
	Super::BeginPlay();

}

void ATestActor::UpdateSpline(const TArray<FVector>& PathPoints)
{
	if (!PathSpline) return;
	if (PathPoints.Num() < 2) return; // 점이 2개 미만이면 스플라인을 만들 수 없음

	// 1. 기존 스플라인 포인트를 모두 지웁니다.
	PathSpline->ClearSplinePoints();

	// 2. 경로의 각 점을 스플라인 포인트로 추가합니다.
	for (const FVector& Point : PathPoints)
	{
		// AddSplinePoint는 마지막에 점을 추가합니다.
		// 좌표 공간은 World로 설정하여 월드 위치를 그대로 사용합니다.
		PathSpline->AddSplinePoint(Point, ESplineCoordinateSpace::World);
	}

	// (선택적) 디버깅을 위해 스플라인을 보이게 설정
	PathSpline->SetDrawDebug(true);
}

// Called every frame
void ATestActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

