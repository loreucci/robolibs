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

#include "resultslist.h"


class ResultsExplorer : public QMainWindow {

    Q_OBJECT

public:
    explicit ResultsExplorer(QWidget* parent = 0);

public slots:
    void changeFolder();
    void connectToDatabase(const QString& folder);
    void exportLogs();
    void deleteLogs();

    // to be removed, hopefully
    void nyi();

protected:
    ResultsList* list;
    QString currentfolder, lastexportdir;

    void closeEvent(QCloseEvent* event) override;

};

#endif // RESULTSEXPLORER_H
