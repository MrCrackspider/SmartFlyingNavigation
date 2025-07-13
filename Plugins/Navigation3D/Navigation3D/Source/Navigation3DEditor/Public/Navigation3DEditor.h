#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

/**
 * Navigation3D 플러그인의 에디터 모듈입니다.
 * 이 모듈은 플러그인과 관련된 에디터 기능(예: 디테일 패널 커스터마이징)을 등록하고 관리합니다.
 */
class FNavigation3DEditorModule : public IModuleInterface
{
public:
    /** IModuleInterface 구현부 */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};