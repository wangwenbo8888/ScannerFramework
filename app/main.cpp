#include "MainWindow.h"
#include <QApplication>
#include <QFile>
#include <QScreen>

int main(int argc, char *argv[])
{
    _putenv_s("OSG_PLUGIN_PATH", "F:/osg3.6.5/install/bin/osgPlugins-3.6.5");
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    QApplication app(argc, argv);

    QFile styleFile(":/icons/dark.qss");
    if (styleFile.open(QFile::ReadOnly | QFile::Text)) {
        app.setStyleSheet(styleFile.readAll());
        styleFile.close();
    }

    MainWindow window;
    QScreen *screen = app.primaryScreen();
    QRect avail = screen->availableGeometry();
    window.setGeometry(avail);
    window.show();

    return app.exec();
}
