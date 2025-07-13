#pragma once
#include "IDetailCustomization.h"

class FNav3DVolumeDetails : public IDetailCustomization
{
public:
	
	// Detail Customization 인스턴스를 생성하는 스태틱 함수 (표준 방식)
	static TSharedRef<IDetailCustomization> MakeInstance();

	// IDetailCustomization 인터페이스의 핵심 함수
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
};