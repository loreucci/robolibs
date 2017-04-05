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
    menuLogs->addAction("Delete selected", this, SLOT(deleteLogs()));
    menuLogs->addSeparator();
    menuLogs->addAction("Select all", list, SLOT(selectall()));
    menuLogs->addAction("Deselect all", list, SLOT(deselectall()));

    setCentralWidget(list);

    currentfolder = QDir::currentPath();
    connectToDatabase(currentfolder);

    lastexportdir = "";

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

    auto l = list->getSelectedList();


    // check for empty fields
    for (int i = 0; i < l.size(); i++) {

        if (l[i].exportname.isEmpty()) {
            QMessageBox::critical(this, "ResultsExplorer", "Some ExportNames are empty!");
            return;
        }

    }

    // choose destination folder
    // select folder with a dialog
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::Directory);
    dialog.setViewMode(QFileDialog::List);
    if (!lastexportdir.isEmpty())
        dialog.setDirectory(lastexportdir);
    else
        dialog.setDirectory(currentfolder);
    QStringList fileNames;
    if (dialog.exec())
        fileNames = dialog.selectedFiles();
    else
        return;
    QString exportdir = fileNames[0];
    lastexportdir = exportdir;

    // copy entries
    for (int i = 0; i < l.size(); i++) {

        // create destination folder if FOLDER_MODE
        if (l[i].mode == 0) {
            QDir dir(exportdir);
            if (!dir.exists(l[i].exportname))
                dir.mkdir(l[i].exportname);
        }

        // create proper suffixes
        QString exportprefix = exportdir + "/" + l[i].exportname;
        QString originalprefix = currentfolder + "/" + l[i].basename + "-" + l[i].timestamp;
        if (l[i].mode == 0) {
            exportprefix += "/";
            originalprefix += "/";
        } else {
            exportprefix += "_";
            originalprefix += "_";
        }

        //copy files
        QStringList files = l[i].filelist.split(",");
        for (int j = 0; j < files.length(); j++) {

            if (QFile::exists(exportprefix + files[j])) {
                QFile::remove(exportprefix + files[j]);
            }

            QFile::copy(originalprefix + files[j], exportprefix + files[j]);

        }

    }

}

void ResultsExplorer::deleteLogs() {

    QMessageBox msgBox;
    msgBox.setText("Deleting the log files is permanent.");
    msgBox.setInformativeText("Are you sure you want to delete them?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);
    int ret = msgBox.exec();
    if (ret == QMessageBox::Cancel)
        return;

    auto l = list->getSelectedList();

    // delete entries
    for (int i = 0; i < l.size(); i++) {

        // remove all dir if FOLDER_MODE, otherwise all files
        if (l[i].mode == 0) {

            QDir dir(currentfolder + "/" + l[i].basename + "-" + l[i].timestamp);
            dir.removeRecursively();

        } else {

            QString prefix = currentfolder + "/" + l[i].basename + "-" + l[i].timestamp + "_";

            QStringList files = l[i].filelist.split(",");
            for (int j = 0; j < files.length(); j++) {
                QFile::remove(prefix + files[j]);
            }

        }

    }

//    list->deleteSelected();

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
