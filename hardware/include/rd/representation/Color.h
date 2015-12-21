//
// Created by arssivka on 12/21/15.
//

#ifndef NAOMECH_COLOR_H
#define NAOMECH_COLOR_H

namespace rd {
    class Color {
    public:
        enum Components {
            RED,
            GREEN,
            BLUE
        };

        Color();

        Color(const char r,
              const char g,
              const char b);

        Color(const char *data);

        inline void setData(const char *data);

        inline void setRed(const char brightness);

        inline void setGreen(const char brightness);

        inline void setBlue(const char brightness);

        inline const char *getData() const;

        inline char getRed() const;

        inline char getGreen() const;

        inline char getBlue() const;

    private:
        char data[3];
    };
}

#endif //NAOMECH_COLOR_H
