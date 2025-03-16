#pragma once

#include "util/ui/SimpleGraphicsObject.hpp"

class Line : public SimpleGraphicsObject {
public:
    Line(RefSerialData::Tx::GraphicColor color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t thickness)
        : SimpleGraphicsObject(color),
          x1(x1),
          y1(y1),
          x2(x2),
          y2(y2),
          thickness(thickness) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) override {
        RefSerialTransmitter::configLine(thickness, x1, y1, x2, y2, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX1 == x1 && prevY1 == y1 && prevX2 == x2 && prevY2 == y2 && prevColor == color && hasDrawn); }

    uint16_t x1, y1, x2, y2, thickness;  // can set this directly, will appear next time drawn

protected:
    void setPrev() {
        prevThickness = thickness;
        prevX1 = x1;
        prevY1 = y1;
        prevX2 = x2;
        prevY2 = y2;
        prevColor = color;
    }

private:
    uint16_t prevThickness, prevX1, prevY1, prevX2, prevY2 = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class UnfilledRectangle : public Line {  // because unfilled rectangle and line store the exact same fields
public:
    UnfilledRectangle(RefSerialData::Tx::GraphicColor graphicColor, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t thickness) : Line(graphicColor, x1, y1, x2, y2, thickness) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) override {
        RefSerialTransmitter::configRectangle(thickness, x1, y1, x2, y2, graphicData);
        setPrev();
    }
};

class UnfilledCircle : public SimpleGraphicsObject {
public:
    UnfilledCircle(RefSerialData::Tx::GraphicColor color, uint16_t cx, uint16_t cy, uint16_t r, uint16_t thickness) : SimpleGraphicsObject(color), cx(cx), cy(cy), r(r), thickness(thickness) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) override {
        RefSerialTransmitter::configCircle(thickness, cx, cy, r, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevCx == cx && prevCy == cy && prevR == r && prevColor == color && hasDrawn); }

    uint16_t cx, cy, r, thickness;  // can set this directly, will appear next time drawn

protected:
    void setPrev() {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevR = r;
        prevColor = color;
    }

private:
    uint16_t prevThickness, prevCx, prevCy, prevR = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class UnfilledEllipse : public SimpleGraphicsObject {
public:
    UnfilledEllipse(RefSerialData::Tx::GraphicColor color, uint16_t cx, uint16_t cy, uint16_t width, uint16_t height, uint16_t thickness)
        : SimpleGraphicsObject(color),
          cx(cx),
          cy(cy),
          width(width),
          height(height),
          thickness(thickness) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) override {
        RefSerialTransmitter::configEllipse(thickness, cx, cy, width, height, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevCx == cx && prevCy == cy && prevWidth == width && prevHeight == height && prevColor == color && hasDrawn); }

    uint16_t cx, cy, width, height, thickness;  // can set this directly, will appear next time drawn

protected:
    void setPrev() {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevWidth = width;
        prevHeight = height;
        prevColor = color;
    }

private:
    uint16_t prevThickness, prevCx, prevCy, prevWidth, prevHeight = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class Arc : public SimpleGraphicsObject {
public:
    Arc(RefSerialData::Tx::GraphicColor color, uint16_t cx, uint16_t cy, uint16_t width, uint16_t height, uint16_t startAngle, uint16_t endAngle, uint16_t thickness)
        : SimpleGraphicsObject(color),
          cx(cx),
          cy(cy),
          width(width),
          height(height),
          startAngle(startAngle),
          endAngle(endAngle),
          thickness(thickness) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) override {
        RefSerialTransmitter::configArc(startAngle, endAngle, thickness, cx, cy, width, height, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevCx == cx && prevCy == cy && prevWidth == width && prevHeight == height && prevColor == color && hasDrawn); }

    uint16_t cx, cy, width, height, startAngle, endAngle, thickness;  // can set this directly, will appear next time drawn

protected:
    void setPrev() {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevWidth = width;
        prevHeight = height;
        prevStartAngle = startAngle;
        prevEndAngle = endAngle;
        prevColor = color;
    }

private:
    uint16_t prevThickness, prevCx, prevCy, prevWidth, prevStartAngle, prevEndAngle, prevHeight = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};