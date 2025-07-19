#pragma once
#include "IDetailCustomization.h"

class FNav3DPathfindingDetails : public IDetailCustomization
{
public:
	
	static TSharedRef<IDetailCustomization> MakeInstance();

	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
	
};
