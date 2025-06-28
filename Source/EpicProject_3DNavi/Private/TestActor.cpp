// Fill out your copyright notice in the Description page of Project Settings.


#include "TestActor.h"

// Sets default values
ATestActor::ATestActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ATestActor::BeginPlay()
{
	Super::BeginPlay();
	OpenSet.HeapPush(5, FCmp());
	OpenSet.HeapPush(8, FCmp());
	OpenSet.HeapPush(15, FCmp());
	OpenSet.HeapPush(9, FCmp());
	OpenSet.HeapPush(4, FCmp());

	int a;
	OpenSet.HeapPop(a);
	
	UE_LOG(LogTemp, Warning, TEXT("%d"), a);
}

// Called every frame
void ATestActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

