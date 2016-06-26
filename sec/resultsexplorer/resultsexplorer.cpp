#include "resultsexplorer.h"

#include <QMenuBar>
#include <QApplication>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QCloseEvent>

#include <QDebug>


const QString defaultname = "/logs.txt";

ResultsExplorer::ResultsExplorer(QWidget* parent) : QMainWindow(parent) {

    list = new ResultsList(this);

    QMenu* menuFile = menuBar()->addMenu("File");
    menuFile->addAction("Change folder", this, SLOT(changeFolder()));
    menuFile->addAction("Quit", this, SLOT(close()));

    QMenu* menuLogs = menuBar()->addMenu("Logs");
    menuLogs->addAction("Save changes", list, SLOT(saveChanges()));
    menuLogs->addAction("Export selected", this, SLOT(exportLogs()));

    setCentralWidget(list);

    currentfolder = QDir::currentPath();
    connectToDatabase(currentfolder);

    this->resize(600, 400);

}

void ResultsExplorer::changeFolder() {

    // select folder with a dialog
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::Directory);
    dialog.setViewMode(QFileDialog::List);
    dialog.setDirectory(currentfolder);
    QStringList fileNames;
    if (dialog.exec())
        fileNames = dialog.selectedFiles();
    else
        return;

    connectToDatabase(fileNames[0]);

}

void ResultsExplorer::connectToDatabase(const QString& folder) {

    // check whether a database exists
    QString filename = folder + defaultname;
    if (!QFile::exists(filename)) {
        QMessageBox::critical(this, "ResultsExplorer", "No database found in "+folder+" !");
        return;
    }
    currentfolder = folder;

    list->changeFile(filename);
    list->loadData();

}

void ResultsExplorer::exportLogs() {

    auto l = list->getExportRequests();


    // check for empty fields
    for (int i = 0; i < l.size(); i++) {

        if (l[i][1].isEmpty()) {
            QMessageBox::critical(this, "ResultsExplorer", "Some ExportNames are empty!");
            return;
        }

    }

    // choose destination folder
    // select folder with a dialog
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::Directory);
    dialog.setViewMode(QFileDialog::List);
    dialog.setDirectory(currentfolder);
    QStringList fileNames;
    if (dialog.exec())
        fileNames = dialog.selectedFiles();
    else
        return;
    QString exportdir = fileNames[0];

    // copy files
    for (int i = 0; i < l.size(); i++) {

        QString exportname = exportdir + "/" + l[i][1];


        // copy files
        if (QFile::exists(exportname)) {
            QFile::remove(exportname);
        }
        QFile::copy(currentfolder + "/" + l[i][0], exportname);

        // parameters
        if (l[i][2] == "YES") {

            // find name and extension
            QStringList parts = l[i][0].split(".");
            QStringList partsexp = l[i][1].split(".");

            // guess parameters name
            QString parameters = currentfolder + "/" + parts[0] + "_parameters";
            for (int j = 1; j < parts.size(); j++) {
                parameters.append(".");
                parameters.append(parts[j]);
            }

            QString parametersexp = exportdir + "/" + partsexp[0] + "_parameters";
            for (int j = 1; j < partsexp.size(); j++) {
                parametersexp.append(".");
                parametersexp.append(partsexp[j]);
            }

            if (QFile::exists(parametersexp)) {
                QFile::remove(parametersexp);
            }
            QFile::copy(parameters, parametersexp);

        }

    }

}


void ResultsExplorer::nyi() {

    QMessageBox::warning(this, "Resultsexplorer", "Not yet implemented.");

}

void ResultsExplorer::closeEvent(QCloseEvent* event) {

    if (list->isChanged()) {
        QMessageBox msgBox;
        msgBox.setText("The document has been modified.");
        msgBox.setInformativeText("Do you want to save your changes?");
        msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Save);
        int ret = msgBox.exec();
        if (ret == QMessageBox::Save) {
            list->saveChanges();
        } else if (ret == QMessageBox::Cancel) {
            event->ignore();
        }
    } else {
        QMainWindow::closeEvent(event);
    }

}
