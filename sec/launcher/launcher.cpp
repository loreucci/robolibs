#include "launcher.h"

#include <QMenuBar>
#include <QApplication>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QCloseEvent>
#include <QFormLayout>
#include <QToolBar>
#include <QLabel>
#include <QTimer>
#include <QTextStream>

#include "dataoutwidget.h"
#include "../sec/launcherinterface.h"

const QString defaultname = "args.txt";
const QString defaultdatasocketarg = QString::fromStdString(sec::launchersocketarg);

Launcher::Launcher(QWidget* parent)
    :QMainWindow(parent) {

    currentstate = CHOOSE;

    QMenu* menuFile = menuBar()->addMenu("File");
    menuFile->addAction("Select file", this, SLOT(selectFile()));
    menuFile->addAction("Quit", this, SLOT(close()));

    openfile(defaultname);
    createContent();

    QToolBar* toolbar = addToolBar("main_toolbar");
    addToolBar(Qt::BottomToolBarArea, toolbar);

    if (currentstate == CHOOSE) {
        startAct = new QAction("Start (gen-only)", this);
    } else {
        startAct = new QAction("Start", this);
    }
    connect(startAct, SIGNAL(triggered(bool)), this, SLOT(execstart(bool)));

    stopAct = new QAction("Stop", this);
    stopAct->setEnabled(false);
    connect(stopAct, SIGNAL(triggered(bool)), this, SLOT(execstop(bool)));

    killAct = new QAction("Kill", this);
    killAct->setEnabled(false);
    connect(killAct, SIGNAL(triggered(bool)), this, SLOT(execkill(bool)));

    toolbar->addAction(startAct);
    toolbar->addAction(stopAct);
    toolbar->addAction(killAct);


    this->resize(600, 400);

    currentfolder = QDir::currentPath();
    lastfile = defaultname;

    // check for changes
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(checkChanged()));
    timer->start(1000);

}

void Launcher::selectFile() {

    // select file
    QFileDialog dialog(this);
    dialog.setViewMode(QFileDialog::List);
    dialog.setDirectory(currentfolder);
    QStringList fileNames;
    if (dialog.exec())
        fileNames = dialog.selectedFiles();
    else
        return;

    // load it and create content
    if (openfile(fileNames[0])) {
        createContent();
        lastfile = fileNames[0];
    }

}

bool Launcher::openfile(QString filename) {

    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }

    QList<Entry> _args;

    QTextStream in(&file);
    int row = 0;
    while (!in.atEnd()) {
        QString line = in.readLine();
        if (row == 0) {
            execname = line;
        } else {
            QStringList lline = line.split(";");
            _args.append({lline[0], lline[1], nullptr});
        }
        row++;
    }

    // just check whether we need to update the gui
    if (currentstate == LAUNCH) {
        QStringList l, _l;
        for (auto& e : args) {
            l << e.name;
        }
        l.sort();
        for (auto& e : _args) {
            _l << e.name;
        }
        _l.sort();
        if (l == _l) {
            return false;
        }

    }

    // first time, new args in LAUNCH or we are back from EXEC
    args.clear();
    args = _args;
    for (auto& e : args) {
        if (e.name == defaultdatasocketarg)
            continue;
        e.edit = new QLineEdit(this);
        e.edit->setText(e.value);
    }

    currentstate = LAUNCH;
    if (startAct != nullptr)
        startAct->setText("Start");
    return true;

}


void Launcher::closeEvent(QCloseEvent* event) {

    if (process != nullptr) {
        QMessageBox msgBox;
        msgBox.setText("A process is still running.");
        msgBox.setInformativeText("Do you want to exit anyway?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        int ret = msgBox.exec();
        if (ret == QMessageBox::Yes) {
            execkill(true);
            QMainWindow::closeEvent(event);
        } else {
            event->ignore();
        }
    } else {
        QMainWindow::closeEvent(event);
    }

}


void Launcher::createContent() {


    if (currentstate == CHOOSE) {

        QWidget* central = new QWidget(this);
        QFormLayout* layout = new QFormLayout(central);
        central->setLayout(layout);

        execedit = new QLineEdit(central);
        layout->addRow("Executable:", execedit);

        if (centralWidget() != nullptr)
            centralWidget()->deleteLater();
        setCentralWidget(central);

    } else if (currentstate == LAUNCH) {

        QWidget* central = new QWidget(this);
        QFormLayout* layout = new QFormLayout(central);
        central->setLayout(layout);

        layout->addRow("Executable:", new QLabel(execname, central));

        for (int i = 0; i < args.size(); i++) {
            if (args[i].name == defaultdatasocketarg)
                continue;
            layout->addRow(args[i].name, args[i].edit);
        }

        if (centralWidget() != nullptr)
            centralWidget()->deleteLater();
        setCentralWidget(central);

    } if (currentstate == EXEC) {

        int i = 0;
        for (; i < args.size(); i++) {
            if (args[i].name == defaultdatasocketarg)
                break;
        }
        if (i < args.size()) {
            DataOutWidget* central = new DataOutWidget(args[i].value, this);
            if (centralWidget() != nullptr)
                centralWidget()->deleteLater();
            setCentralWidget(central);
        }

    }

}

void Launcher::processStarted() {
    timer->stop();
    if (currentstate != CHOOSE)
        currentstate = EXEC;
    startAct->setEnabled(false);
    stopAct->setEnabled(true);
    killAct->setEnabled(true);
    createContent();
}

void Launcher::processEnded(int, QProcess::ExitStatus) {
    startAct->setEnabled(true);
    stopAct->setEnabled(false);
    killAct->setEnabled(false);
    process->deleteLater();
    process = nullptr;
    timer->start(1000);
    currentstate = CHOOSE;
    checkChanged();
}

void Launcher::processError(QProcess::ProcessError error) {
    qCritical() << "An error occurred: " << error;
    process->deleteLater();
    process = nullptr;
}

void Launcher::processOutput() {
    printf("%s", process->readAllStandardOutput().constData());
}

void Launcher::execstart(bool) {

    if (currentstate == EXEC)
        return;

    // get args
    QStringList arglist;
    if (currentstate == CHOOSE) {
        arglist << "--gen-only";
    } else {
        for (const auto& a : args) {
            if (a.name == defaultdatasocketarg)
                continue;
            arglist << "--" + a.name;
            arglist << a.edit->text();
        }
    }

    // spawn process
    process = new QProcess(this);
    process->setProcessChannelMode(QProcess::MergedChannels);

    connect(process, SIGNAL(started()), this, SLOT(processStarted()));
    connect(process, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(processEnded(int,QProcess::ExitStatus)));
    connect(process, SIGNAL(errorOccurred(QProcess::ProcessError)), this, SLOT(processError(QProcess::ProcessError)));
    connect(process, SIGNAL(readyReadStandardOutput()), this, SLOT(processOutput()));
    connect(process, SIGNAL(readyReadStandardError()), this, SLOT(processOutput()));

    if (currentstate == CHOOSE) {
        process->start(execedit->text(), arglist);
    } else {
        process->start(execname, arglist);
    }

}

void Launcher::execstop(bool) {
    if (process != nullptr) {
        process->terminate();
    }
}

void Launcher::execkill(bool) {
    if (process != nullptr) {
        process->kill();
    }
}

void Launcher::checkChanged() {

    if (openfile(lastfile)) {
        createContent();
    }

}
