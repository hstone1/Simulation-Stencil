#ifndef CLSETTINGS_H
#define CLSETTINGS_H

#include <QString>
class ConfigStore
{
public:
    static ConfigStore& get()
    {
        static ConfigStore instance;
        return instance;
    }
    static const char *getMeshFilename() {
        return get().filename.toUtf8().constData();
    }
    void setMeshFilename(QString str) {
        filename = str;
    }
    void setRenderingSetting(QString filename) {
        _rendering = true;
        _renderDir = filename;
    }
    void setNotRendering() {
        _rendering = false;
    }
    void setSpeedFactor(float speed) {
        speedFactor = speed;
    }

    static float getSpeedFactor() {
        return get().speedFactor;
    }

    static bool rendering() {
        return get()._rendering;
    }

    static QString renderDir() {
        return get()._renderDir;
    }

private:
    ConfigStore(){};
    QString filename;
    float speedFactor;
    bool _rendering;
    QString _renderDir;
};

#endif // CLSETTINGS_H
