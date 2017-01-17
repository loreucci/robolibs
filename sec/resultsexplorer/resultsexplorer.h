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
