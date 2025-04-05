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

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        RefSerialTransmitter::configLine(thickness, x1, y1, x2, y2, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX1 == x1 && prevY1 == y1 && prevX2 == x2 && prevY2 == y2 && prevColor == color && hasDrawn); }

    uint16_t x1, y1, x2, y2, thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev() {
        prevThickness = thickness;
        prevX1 = x1;
        prevY1 = y1;
        prevX2 = x2;
        prevY2 = y2;
        prevColor = color;
    }

    uint16_t prevThickness, prevX1, prevY1, prevX2, prevY2 = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class UnfilledRectangle : public SimpleGraphicsObject {
public:
    UnfilledRectangle(RefSerialData::Tx::GraphicColor color, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t thickness)
        : SimpleGraphicsObject(color),
          x(x),
          y(y),
          width(width),
          height(height),
          thickness(thickness) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        RefSerialTransmitter::configRectangle(thickness, x, y, width + x, height + y, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX == x && prevY == y && prevWidth == width && prevHeight == height && prevColor == color && hasDrawn); }

    uint16_t x, y, width, height, thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev() {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevWidth = width;
        prevHeight = height;
        prevColor = color;
    }

    uint16_t prevThickness, prevX, prevY, prevWidth, prevHeight = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class UnfilledCircle : public SimpleGraphicsObject {
public:
    UnfilledCircle(RefSerialData::Tx::GraphicColor color, uint16_t cx, uint16_t cy, uint16_t r, uint16_t thickness) : SimpleGraphicsObject(color), cx(cx), cy(cy), r(r), thickness(thickness) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        RefSerialTransmitter::configCircle(thickness, cx, cy, r, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevCx == cx && prevCy == cy && prevR == r && prevColor == color && hasDrawn); }

    uint16_t cx, cy, r, thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev() {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevR = r;
        prevColor = color;
    }

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

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        RefSerialTransmitter::configEllipse(thickness, cx, cy, width, height, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevCx == cx && prevCy == cy && prevWidth == width && prevHeight == height && prevColor == color && hasDrawn); }

    uint16_t cx, cy, width, height, thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev() {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevWidth = width;
        prevHeight = height;
        prevColor = color;
    }

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

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        RefSerialTransmitter::configArc(startAngle, endAngle, thickness, cx, cy, width, height, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevCx == cx && prevCy == cy && prevWidth == width && prevHeight == height && prevColor == color && prevStartAngle == startAngle&&prevEndAngle == endAngle&&hasDrawn); }

    uint16_t startAngle, endAngle;  // can set this directly, will appear next time drawn, 0 is up, positive is clockwise, in degrees
    uint16_t cx, cy, width, height, thickness;  // can set this directly, will appear next time drawn

private:
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

    uint16_t prevThickness, prevCx, prevCy, prevWidth, prevStartAngle, prevEndAngle, prevHeight = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class TextSizer {
public:
    uint16_t height, x, y = 0;  // can set this directly, will appear next time drawn
    uint16_t width = 0;         // can read this, setting will be in vain, reset next time drawn

    TextSizer(uint16_t len) : len(len) {}
    TextSizer(uint16_t len, uint16_t x, uint16_t y, uint16_t height) : height(height), x(x), y(y), len(len) {}

    /* resizes this text to fit within the given rect, ignoring width because it doesn't cut off the contained text */
    void inputRect(UnfilledRectangle* rect) {
        x = rect->x;
        y = rect->y;
        height = rect->height;
        calculateNumbers();  // caller might want to have an updated width
    }

    /* resizes the given rect to bound this text, will include width */
    void outputRect(UnfilledRectangle* rect) {
        calculateNumbers();
        rect->x = x;
        rect->y = y;
        rect->width = width;
        rect->height = height;
    }

private:

    // need to test/tune, was from ui website
    static constexpr uint16_t WIDTH_OFFSET_MULT = 19;
    static constexpr uint16_t WIDTH_OFFSET_DIV = 47;

    void calculateWidth() { width = fontSize * len - fontSize*WIDTH_OFFSET_MULT / WIDTH_OFFSET_DIV; }

protected:
    uint16_t fontSize, textX, textY = 0;  // can read these, but don't set these, set with setTextNumbers
    uint16_t len = 0;                     // for sending integer 123 or text ABC, len would be 3. Not sure about floats yet, need to test

    void calculateNumbers() {
        fontSize = height;
        textX = x;
        textY = y + height;
        calculateWidth();
    }

    uint16_t intLen(int32_t n) {
        if (n == 0) return 1;
        if (n < 0) return 1 + intLen(-n);
        return std::floor(std::log10(n) + 1);
    }

    // assumes 1.2 will be shown 1.200; 4 as 4.000; and 1.11111 as 1.111, need to test
    uint16_t floatLen(float n) {
        if (n >= 0 && n < 1) return 5;
        if (n < 0) return 1 + floatLen(-n);
        return std::floor(std::log10(n) + 5);
    }

    // assumes null terminated, also strings longer than 30 will say size 30 because only 30 can be send in one message
    uint16_t stringLen(const char* str) {
        uint16_t r = strlen(str);
        return r < 30 ? r : 30;
    }

    void setLen(uint16_t newLen) {
        len = newLen;
        calculateWidth();
    }
};

class IntegerGraphic : public SimpleGraphicsObject, public TextSizer {
public:
    IntegerGraphic(int32_t newInteger, UnfilledRectangle* rect) : SimpleGraphicsObject(rect->color), TextSizer(intLen(newInteger)), thickness(rect->thickness), integer(newInteger) { inputRect(rect); }

    IntegerGraphic(RefSerialData::Tx::GraphicColor color, int32_t newInteger, uint16_t x, uint16_t y, uint16_t height, uint16_t thickness)
        : SimpleGraphicsObject(color),
          TextSizer(intLen(newInteger), x, y, height),
          thickness(thickness) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        setLen(intLen(integer));
        calculateNumbers();
        RefSerialTransmitter::configInteger(fontSize, thickness, textX, textY, integer, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX == x && prevY == y && prevHeight == height && prevColor == color && prevInteger == integer && hasDrawn); }

    uint16_t thickness = 0;
    int32_t integer = 0;

private:
    void setPrev() {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevHeight = height;
        prevInteger = integer;
        prevColor = color;
    }

    uint16_t prevX, prevY, prevHeight, prevThickness = 0;
    int32_t prevInteger = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class FloatGraphic : public SimpleGraphicsObject, public TextSizer {
public:
    FloatGraphic(float newFloat, UnfilledRectangle* rect) : SimpleGraphicsObject(rect->color), TextSizer(floatLen(newFloat)), thickness(rect->thickness), _float(newFloat) { inputRect(rect); }

    FloatGraphic(RefSerialData::Tx::GraphicColor color, float newFloat, uint16_t x, uint16_t y, uint16_t height, uint16_t thickness)
        : SimpleGraphicsObject(color),
          TextSizer(floatLen(newFloat), x, y, height),
          thickness(thickness) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        setLen(floatLen(_float));
        calculateNumbers();
        // the 3 is decimal precision, need to see what changing it does
        RefSerialTransmitter::configFloatingNumber(fontSize, 3, thickness, textX, textY, _float, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX == x && prevY == y && prevHeight == height && prevColor == color && prevFloat == _float && hasDrawn); }

    uint16_t thickness = 0;
    float _float = 0;

private:
    void setPrev() {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevHeight = height;
        prevFloat = _float;
        prevColor = color;
    }

    uint16_t prevX, prevY, prevHeight, prevThickness = 0;
    float prevFloat = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class StringGraphic : public SimpleGraphicsObject, public TextSizer {
private:
    static constexpr int STRING_SIZE = 31; //not sure if it should be 30 or 31

public:
    StringGraphic(const char* newString, UnfilledRectangle* rect) : SimpleGraphicsObject(rect->color), TextSizer(stringLen(newString)), thickness(rect->thickness) {
        inputRect(rect);
        setString(newString);
    }

    StringGraphic(RefSerialData::Tx::GraphicColor color, const char* newString, uint16_t x, uint16_t y, uint16_t height, uint16_t thickness)
        : SimpleGraphicsObject(color),
          TextSizer(stringLen(newString), x, y, height),
          thickness(thickness) {
        setString(newString);
    }

    void setString(const char* newString) {
        strncpy(string, newString, STRING_SIZE);
    }

    void configCharacterData(RefSerialData::Tx::GraphicCharacterMessage* characterData) final {
        setLen(stringLen(string));
        calculateNumbers();
        configGraphicData(&characterData->graphicData);
        RefSerialTransmitter::configCharacterMsg(fontSize, thickness, textX, textY, string, characterData);
        setPrev();
    }

    // StringGraphics fill the data differently. configGraphicGenerics still needs called, but finishConfigGraphicData shouldn't do anything extra
    void finishConfigGraphicData(__attribute__((unused)) RefSerialData::Tx::GraphicData* graphicData) final {}

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX == x && prevY == y && prevHeight == height && prevColor == color && !std::strncmp(string, oldString, STRING_SIZE) && hasDrawn); }

    uint16_t thickness = 0;
    char string[STRING_SIZE];

    bool isStringGraphic() final { return true; }

private:
    void setPrev() {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevHeight = height;
        prevColor = color;
        strncpy(oldString, string, STRING_SIZE);
    }

    uint16_t prevX, prevY, prevHeight, prevThickness = 0;
    char oldString[STRING_SIZE];
    RefSerialData::Tx::GraphicColor prevColor;

};