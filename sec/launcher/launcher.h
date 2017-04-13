#ifndef RESULTSEXPLORER_H
#define RESULTSEXPLORER_H

#include <QMainWindow>
#include <QLineEdit>
#include <QProcess>


class Launcher : public QMainWindow {

    Q_OBJECT

public:
    explicit Launcher(QWidget* parent = 0);

public slots:
    void selectFile();
    void execstart(bool);
    void execstop(bool);
    void execkill(bool);
    void checkChanged();

protected:
    bool openfile(QString filename);

    void closeEvent(QCloseEvent* event) override;

    void createContent();

    struct Entry {
        QString name;
        QString value;
        QLineEdit* edit;
    };

    QString currentfolder, lastfile;
    QList<Entry> args;
    QString execname;
    QProcess* process = nullptr;

    // stuff for initialization
    bool initialized;
    QLineEdit* execedit = nullptr;

    QAction *startAct = nullptr, *stopAct = nullptr, *killAct = nullptr;

protected slots:
    void processStarted();
    void processEnded(int, QProcess::ExitStatus);
    void processError(QProcess::ProcessError error);
    void processOutput();

};

#endif // RESULTSEXPLORER_H
