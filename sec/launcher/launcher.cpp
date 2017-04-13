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

#include <QDebug>


const QString defaultname = "args.txt";

Launcher::Launcher(QWidget* parent)
    :QMainWindow(parent) {

    initialized = false;

    QMenu* menuFile = menuBar()->addMenu("File");
    menuFile->addAction("Select file", this, SLOT(selectFile()));
    menuFile->addAction("Quit", this, SLOT(close()));

    openfile(defaultname);
    createContent();

    QToolBar* toolbar = addToolBar("main_toolbar");
    addToolBar(Qt::BottomToolBarArea, toolbar);

    if (!initialized) {
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
    QTimer* timer = new QTimer(this);
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
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;

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

    QStringList l, _l;
    for (auto& e : args) {
        l << e.name;
    }
    l.sort();
    for (auto& e : _args) {
        _l << e.name;
    }
    _l.sort();
    if (l == _l)
        return false;

    args.clear();
    args = _args;
    for (auto& e : args) {
        e.edit = new QLineEdit(this);
        e.edit->setText(e.value);
    }

    initialized = true;
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

    QWidget* central = new QWidget(this);

    QFormLayout* layout = new QFormLayout(central);
    central->setLayout(layout);

    if (!initialized) {

        execedit = new QLineEdit(central);
        layout->addRow("Executable:", execedit);

    } else {

        layout->addRow("Executable:", new QLabel(execname, central));

        for (int i = 0; i < args.size(); i++) {
            layout->addRow(args[i].name, args[i].edit);
        }
    }

    if (centralWidget() != nullptr)
        centralWidget()->deleteLater();
    setCentralWidget(central);

}

void Launcher::processStarted() {
    startAct->setEnabled(false);
    stopAct->setEnabled(true);
    killAct->setEnabled(true);
}

void Launcher::processEnded(int, QProcess::ExitStatus) {
    startAct->setEnabled(true);
    stopAct->setEnabled(false);
    killAct->setEnabled(false);
    process->deleteLater();
    process = nullptr;
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

    // get args
    QStringList arglist;
    if (!initialized) {
        arglist << "--gen-only";
    } else {
        for (auto& a : args) {
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

    if (!initialized) {
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
