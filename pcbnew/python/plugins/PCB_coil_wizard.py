#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#
from __future__ import division

import math

import pcbnew
import FootprintWizardBase

def flat_spiral_coil_depth(L, ro, N):
    """Calculate the depth of a flat spiral coil.

    Keyword arguments:
    L    -- inductance in uH
    ro   -- outer radius of coil in cm
    N    -- number of turns
    """
    # code based on formula two and figure 3 in https://sci-hub.tw/https://ieeexplore.ieee.org/document/1669896
    # and on https://en.wikipedia.org/wiki/Inductor#Inductance_formulas
    # and on https://www.wolframalpha.com/input/?i=solve+L+%3D+((r-d%2F2)%5E2N%5E2)%2F(20(r-d%2F2)%2B28d)+for+d
    ro_cm = ro/10
    d_cm = (2*(-2*math.sqrt(81*L*L+14*L*N*N*ro_cm)+18*L+N*N*ro_cm))/(N*N)
    return d_cm*10

class PCB_coil_wizard(FootprintWizardBase.FootprintWizard):


    def GetName(self):
        return "PCB Coil"

    def GetDescription(self):
        return "Spiral PCB coil"

    def GenerateParameterList(self):
                    # page, label, data type, default value
        self.AddParam("Properties", "inductance", self.uFloat, 1, min_value=0)
        self.AddParam("Properties", "outer radius", self.uMM, 10, min_value=0)
        self.AddParam("Properties", "turns", self.uInteger, 10)

        self.AddParam("Trace", "trace width", self.uMM, 0.1, min_value=0)
        self.AddParam("Trace", "clockwise", self.uBool, True, hint="Rotation direction going inwards")
        self.AddParam("Trace", "doublesided", self.uBool, False)
        self.AddParam("Trace", "segments", self.uInteger, 90, hint="Segmens per rotation")
        self.AddParam("Trace", "via drill", self.uMM, 0.3, min_value=0)

        self.AddParam("Pads", "diameter", self.uMM, 0.1, min_value=0)

        self.AddParam("Outline", "diameter", self.uMM, 7, designator='D')
        self.AddParam("Outline", "margin", self.uMM, 0.25, min_value=0.2)

    def CheckParameters(self):

        properties = self.parameters['Properties']
        trace = self.parameters['Trace']

        # Check that depth of coil is smaller than or equal to outer radius
        L = properties['inductance']
        ro = pcbnew.ToMM(properties['outer radius'])
        N = properties['turns']
        doublesided = trace['doublesided']
        L = L/2 if doublesided else L
        d = flat_spiral_coil_depth(L, ro, N)

        self.CheckParam('Properties', 'outer radius', min_value=d, info="Inductance to small or too many turns")


    def GetValue(self):
        inductance = self.parameters["Properties"]["inductance"]
        return "L_%.2fuH" % inductance

    def TraceSegmentPad(self, start, end, width, layer=pcbnew.F_Cu):
        # Calculate length and angle
        x0 = start.x
        y0 = start.y
        x1 = end[0]
        y1 = end[1]
        dx = end.x-start.x
        dy = end.y-start.y
        length = math.sqrt(dx*dx + dy*dy)+width
        size = pcbnew.wxSize(length, width)
        pos = pcbnew.wxPoint((start.x+end.x)/2, (start.y+end.y)/2)

        # transform pith into degrees
        rot_degree = 0
        if dx == 0:
            rot_degree = 90
        else:
            rot_degree = -math.atan(dy/dx)/math.pi/2*360

        pad = pcbnew.D_PAD(self.module)
        pad.SetSize(size)
        pad.SetShape(pcbnew.PAD_SHAPE_OVAL)
        pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        pad.SetLayerSet(pcbnew.LSET(layer))     # copper layer only
        pad.SetPos0(pos)
        pad.SetPosition(pos)
        pad.SetOrientation(rot_degree*10)   # rotation is in 0.1 degrees
        return pad

    def SpiralTurn(self, ro, d, segments, widht, layer=pcbnew.F_Cu, cw=True):
        points = [pcbnew.wxPoint(
            (ro - d/segments*i) * math.cos(2*math.pi*i/segments),
            (ro - d/segments*i) * math.sin(2*math.pi*i/segments)*(1 if cw else -1))
            for i in range(segments+1)]

        for (p0, p1) in zip(points[1:], points[:-1]):
            self.module.Add(self.TraceSegmentPad(p0, p1, widht, layer))

    def THPad(self, Vsize, Hsize, drill, shape=pcbnew.PAD_SHAPE_OVAL,
              rot_degree = 0):
        """!
        A basic through-hole pad of the given size and shape
        @param Vsize: the vertical size of the pad
        @param Hsize: the horizontal size of the pad
        @param drill: the drill diameter
        @param shape: the shape of the pad
        @param rot_degree: the pad rotation, in degrees
        """
        pad = pcbnew.D_PAD(self.module)
        pad.SetSize(pcbnew.wxSize(Hsize, Vsize))
        pad.SetShape(shape)
        pad.SetAttribute(pcbnew.PAD_ATTRIB_STANDARD)
        pad.SetLayerSet(pad.StandardMask())
        pad.SetDrillSize(pcbnew.wxSize(drill, drill))
        pad.SetOrientation(rot_degree*10)   # rotation is in 0.1 degrees

        return pad

    def SpiralPad(self, xpos, width, layer, name):
        pad = pcbnew.D_PAD(self.module)
        pos = pcbnew.wxPoint(xpos, 0)
        pad.SetSize(pcbnew.wxSize(width, width))
        pad.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
        pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        pad.SetLayerSet(pcbnew.LSET(layer))     # copper layer only
        pad.SetPos0(pos)
        pad.SetPosition(pos)
        pad.SetName(name)
        return pad

    def DoubleSpiralVia(self, xpos, width, drill):
        pos = pcbnew.wxPoint(xpos, 0)
        # via = pcbnew.VIA(self.module)
        # via.SetDrill(drill)
        # via.SetPosition(pos)
        # via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        pad = pcbnew.D_PAD(self.module)
        pad.SetSize(pcbnew.wxSize(drill+2*width, drill+2*width))
        pad.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
        pad.SetAttribute(pcbnew.PAD_ATTRIB_STANDARD)
        pad.SetLayerSet(pad.StandardMask())
        pad.SetDrillSize(pcbnew.wxSize(drill, drill))
        pad.SetPos0(pos)
        pad.SetPosition(pos)
        return pad

    def BuildThisFootprint(self):
        properties = self.parameters['Properties']
        trace = self.parameters['Trace']
        outline = self.parameters['Outline']

        L = properties['inductance']
        ro = properties['outer radius']
        N = properties['turns']

        segments = trace['segments']
        clockwise = trace['clockwise']
        widht = trace['trace width']
        doublesided = trace['doublesided']
        drill = trace['via drill']

        L = L/2 if doublesided else L

        d = pcbnew.FromMM(flat_spiral_coil_depth(L, pcbnew.ToMM(ro), N))

        for turn in range(N):
            self.SpiralTurn(ro-turn*d/N, d/N, segments, widht, pcbnew.F_Cu, clockwise)

        self.module.Add(self.SpiralPad(ro, widht, pcbnew.F_Cu, "1"))

        if doublesided:
            for turn in range(N):
                self.SpiralTurn(ro-turn*d/N, d/N, segments, widht, pcbnew.B_Cu, not clockwise)
            self.module.Add(self.SpiralPad(ro, widht, pcbnew.B_Cu, "2"))
            self.module.Add(self.DoubleSpiralVia(ro-d, widht, drill))
        else:
            self.module.Add(self.SpiralPad(ro-d, widht, pcbnew.F_Cu, "2"))

        # Draw the outline
        self.draw.SetLayer(pcbnew.F_Fab)
        self.draw.GetLineThickness()
        self.draw.Circle(0, 0, ro)

        #silkscreen
        ro += pcbnew.FromMM(0.15)
        self.draw.SetLayer(pcbnew.F_SilkS)
        self.draw.Circle(0, 0, ro)

        # courtyard
        self.draw.SetLayer(pcbnew.F_CrtYd)
        self.draw.SetLineThickness(pcbnew.FromMM(0.05))
        self.draw.Circle(0, 0, ro + outline['margin'])

        # Text size

        text_size = self.GetTextSize()  # IPC nominal
        thickness = self.GetTextThickness()
        textposy = ro + self.draw.GetLineThickness()/2 + self.GetTextSize()/2 + thickness + outline['margin']
        self.draw.Value(0, textposy, text_size)
        self.draw.Reference(0, 0, text_size)

PCB_coil_wizard().register()
