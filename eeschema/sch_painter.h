/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2014 CERN
 * @author Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search the http://www.gnu.org website for the version 2 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#ifndef __SCH_PAINTER_H
#define __SCH_PAINTER_H

#include <painter.h>

class LIB_RECTANGLE;
class LIB_PIN;
class LIB_CIRCLE;
class LIB_ITEM;
class LIB_PART;
class LIB_ALIAS;
class LIB_POLYLINE;
class LIB_ARC;
class LIB_FIELD;
class LIB_TEXT;
class SCH_COMPONENT;
class SCH_FIELD;
class SCH_JUNCTION;
class SCH_LABEL;
class SCH_TEXT;
class SCH_HIERLABEL;
class SCH_GLOBALLABEL;
class SCH_SHEET;
class SCH_SHEET_PIN;
class SCH_MARKER;
class SCH_NO_CONNECT;
class SCH_LINE;
class SCH_BUS_ENTRY_BASE;
class SCH_BITMAP;

namespace KIGFX
{
class GAL;
class SCH_PAINTER;


/**
 * Class SCH_RENDER_SETTINGS
 * Stores schematic-specific render settings.
 */

class SCH_RENDER_SETTINGS : public RENDER_SETTINGS
{
public:
    friend class SCH_PAINTER;

    SCH_RENDER_SETTINGS();

    /// @copydoc RENDER_SETTINGS::ImportLegacyColors()
    void ImportLegacyColors( const COLORS_DESIGN_SETTINGS* aSettings ) override;

    /// @copydoc RENDER_SETTINGS::GetColor()
    virtual const COLOR4D& GetColor( const VIEW_ITEM* aItem, int aLayer ) const override;

    bool IsBackgroundDark() const override
    {
        auto luma = m_layerColors[ LAYER_SCHEMATIC_BACKGROUND ].GetBrightness();

        return luma < 0.5;
    }

    const COLOR4D& GetBackgroundColor() override { return m_layerColors[ LAYER_SCHEMATIC_BACKGROUND ]; }

    const COLOR4D& GetGridColor() override { return m_layerColors[ LAYER_SCHEMATIC_GRID ]; }

    const COLOR4D& GetCursorColor() override { return m_layerColors[ LAYER_SCHEMATIC_CURSOR ]; }

    int  m_ShowUnit;                // Show all units if 0
    int  m_ShowConvert;             // Show all conversions if 0

    bool m_ShowHiddenText;
    bool m_ShowHiddenPins;
    bool m_ShowPinsElectricalType;
};


/**
 * Class SCH_PAINTER
 * Contains methods for drawing schematic-specific items.
 */
class SCH_PAINTER : public PAINTER
{
public:
    SCH_PAINTER( GAL* aGal );

    /// @copydoc PAINTER::Draw()
    virtual bool Draw( const VIEW_ITEM*, int ) override;

    /// @copydoc PAINTER::ApplySettings()
    virtual void ApplySettings( const RENDER_SETTINGS* aSettings ) override
    {
        m_schSettings = *static_cast<const SCH_RENDER_SETTINGS*>( aSettings );
    }

    /// @copydoc PAINTER::GetSettings()
    virtual SCH_RENDER_SETTINGS* GetSettings() override
    {
        return &m_schSettings;
    }

private:
	void draw( LIB_RECTANGLE *, int );
	void draw( LIB_PIN *, int, bool isDangling = true, bool isMoving = false );
	void draw( LIB_CIRCLE *, int );
	void draw( LIB_ITEM *, int );
	void draw( LIB_PART *, int, bool aDrawFields = true,  int aUnit = 0, int aConvert = 0,
               std::vector<bool>* danglingPinFlags = nullptr );
    void draw( LIB_ALIAS *, int );
    void draw( LIB_ARC *, int );
    void draw( LIB_POLYLINE *, int );
    void draw( LIB_FIELD *, int );
    void draw( LIB_TEXT *, int );
    void draw( SCH_COMPONENT *, int );
    void draw( SCH_JUNCTION *, int );
    void draw( SCH_FIELD *, int );
    void draw( SCH_TEXT *, int );
    void draw( SCH_LABEL *, int );
    void draw( SCH_HIERLABEL *, int );
    void draw( SCH_GLOBALLABEL *, int );
    void draw( SCH_SHEET *, int );
    void draw( SCH_SHEET_PIN *, int );
    void draw( SCH_NO_CONNECT *, int );
    void draw( SCH_MARKER *, int );
    void draw( SCH_BITMAP *, int );
    void draw( SCH_LINE *, int );
    void draw( SCH_BUS_ENTRY_BASE *aEntry, int aLayer );

    bool isUnitAndConversionShown( const LIB_ITEM* aItem );

    bool setColors( const LIB_ITEM* aItem, int aLayer );

    void triLine ( const VECTOR2D &a, const VECTOR2D &b, const VECTOR2D &c );

    SCH_RENDER_SETTINGS m_schSettings;
};

}; // namespace KIGFX


#endif
