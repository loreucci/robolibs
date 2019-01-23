/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

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
    enum State {CHOOSE, LAUNCH, EXEC};
    State currentstate;
    QLineEdit* execedit = nullptr;

    QTimer* timer;

    QAction *startAct = nullptr, *stopAct = nullptr, *killAct = nullptr;

protected slots:
    void processStarted();
    void processEnded(int, QProcess::ExitStatus);
    void processError(QProcess::ProcessError error);
    void processOutput();

};

#endif // RESULTSEXPLORER_H
