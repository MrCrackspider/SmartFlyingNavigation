// Copyright © 2025 TeamHamDev. All Rights Reserved.


#pragma once
#include "IDetailCustomization.h"

class FNav3DVolumeDetails : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();

	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
};