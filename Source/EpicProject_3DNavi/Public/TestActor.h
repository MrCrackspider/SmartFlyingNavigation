// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TestActor.generated.h"

struct FPathNode;

UCLASS()
class EPICPROJECT_3DNAVI_API ATestActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ATestActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
    TArray<int> OpenSet;
};

struct FCmp
{
	FORCEINLINE bool operator()(const int A, const int B) const
	{
		return A < B;
		// FIX-ME: Minheap�̶� A->FCost > B->FCost�� �Ǵ°� ������, �޸���ƽ�Լ��� �� �� ���� ������ �����Ǿ�� �������� ��ΰ� ����.
		//return A->FCost > B->FCost;
	}
};