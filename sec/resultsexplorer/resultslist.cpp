#include "resultslist.h"

#include <QHeaderView>
#include <QFile>
#include <QTextStream>

FileModel::FileModel(QObject *parent)
    :QStandardItemModel(parent) {

    changed = false;

    connect(this, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(checkForChange(QStandardItem*)));

}

Qt::ItemFlags FileModel::flags(const QModelIndex& index) const {
    if (index.column() >= 1 && index.column() <= 4)
        return Qt::ItemIsSelectable | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled;
    return QStandardItemModel::flags(index);
}

bool FileModel::isChanged(){
    return changed;
}

void FileModel::resetChange() {
    changed = false;
}

void FileModel::checkForChange(QStandardItem* item) {

    if (!changed && indexFromItem(item).column() > 0)
        changed = true;

}



ResultsList::ResultsList(QWidget* parent) : QTableView(parent) {

    data = new FileModel();

    this->setModel(data);
    this->horizontalHeader()->setStretchLastSection(true);


}

void ResultsList::changeFile(QString datafile) {
    this->datafile = datafile;
}

bool ResultsList::isChanged() {
    return data->isChanged();
}

QList<ExportableEntry> ResultsList::getSelectedList() {

    QList<ExportableEntry> ret;
    for (int r = 0; r < data->rowCount(); r++) {
        if (data->itemFromIndex(data->index(r, 0))->checkState() == Qt::Checked) {
            ExportableEntry entry;
            entry.mode = data->data(data->index(r, 1)).toInt();
            entry.basename = data->data(data->index(r, 2)).toString();
            entry.timestamp = data->data(data->index(r, 3)).toString();
            entry.filelist = data->data(data->index(r, 4)).toString();
            entry.exportname = data->data(data->index(r, 6)).toString();
            ret.append(entry);
        }
    }

    return ret;

}

void ResultsList::loadData() {

    if (data->columnCount() == 0) {
        loadFromFile();
        if (timer == nullptr) {
            timer = new QTimer(this);
            connect(timer, SIGNAL(timeout()), this, SLOT(addNewData()));
            timer->start(1000);
        }
        return;
    }

    // if file is not changed return

    data->clear();
    loadFromFile();

    if (timer == nullptr) {
        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(addNewData()));
        timer->start(1000);
    }

}

void ResultsList::addNewData() {

    QFile file(datafile);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        throw std::runtime_error("Unable to open file.");

    bool itemschanged = data->isChanged();

    QTextStream in(&file);
    int row = 0;
    while (!in.atEnd()) {
        QString line = in.readLine();
        if (row < data->rowCount()) {
            row++;
            continue;
        }
        parseLine(line, row);
        QStandardItem* item = new QStandardItem();
        item->setCheckable(true);
        item->setCheckState(Qt::Unchecked);
        data->setItem(row, 0, item);
        row++;
    }

    file.close();

    if (!itemschanged)
        data->resetChange();

}

void ResultsList::saveChanges() {

    QFile file(datafile);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        throw std::runtime_error("Unable to open file.");

    QTextStream out(&file);
    for (int r = 0; r < data->rowCount(); r++) {
        int c = 1;
        for (; c < data->columnCount()-1; c++) {
            out << data->data(data->index(r, c)).toString() << ";";
        }
        out << data->data(data->index(r, c)).toString() << "\n";
    }

    file.close();

    data->resetChange();

}

void ResultsList::deleteSelected() {

    QList<int> selected;
    for (int r = 0; r < data->rowCount(); r++) {
        if (data->itemFromIndex(data->index(r, 0))->checkState() == Qt::Checked) {
            selected.push_back(r);
        }
    }

    for (int i = selected.length()-1; i >= 0; i--)
        data->removeRow(selected[i]);

    saveChanges();

}

void ResultsList::selectall() {

    for (int r = 0; r < data->rowCount(); r++) {
        data->itemFromIndex(data->index(r, 0))->setCheckState(Qt::Checked);
    }

}

void ResultsList::deselectall() {

    for (int r = 0; r < data->rowCount(); r++) {
        data->itemFromIndex(data->index(r, 0))->setCheckState(Qt::Unchecked);
    }

}

void ResultsList::loadFromFile() {

    QFile file(datafile);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        throw std::runtime_error("Unable to open file.");

    QTextStream in(&file);
    int row = 0;
    while (!in.atEnd()) {
        QString line = in.readLine();
        parseLine(line, row);
        QStandardItem* item = new QStandardItem();
        item->setCheckable(true);
        item->setCheckState(Qt::Unchecked);
        data->setItem(row, 0, item);
        row++;
    }

    data->setHeaderData(0, Qt::Horizontal, "");
    data->setHeaderData(1, Qt::Horizontal, "M");
    data->setHeaderData(2, Qt::Horizontal, "Name");
    data->setHeaderData(3, Qt::Horizontal, "Timestamp");
    data->setHeaderData(4, Qt::Horizontal, "Files");
    data->setHeaderData(5, Qt::Horizontal, "Comment");
    data->setHeaderData(6, Qt::Horizontal, "ExportName");

    file.close();

    data->resetChange();
    this->resizeColumnsToContents();

}

void ResultsList::parseLine(const QString& line, int row) {

    QStringList lline = line.split(";");
    for (int i = 0; i < lline.size(); i++) {
        QStandardItem* item = new QStandardItem(lline[i]);
        data->setItem(row, i+1, item);
    }

}

