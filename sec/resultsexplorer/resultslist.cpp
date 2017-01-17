#include "resultslist.h"

#include <QHeaderView>
#include <QFile>
#include <QTextStream>

#include <QDebug>

FileModel::FileModel(QObject *parent)
    :QStandardItemModel(parent) {

    changed = false;

    connect(this, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(checkForChange(QStandardItem*)));

}

Qt::ItemFlags FileModel::flags(const QModelIndex& index) const {
    if (index.column() == 1 || index.column() == 2)
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

//DataProxy::DataProxy(QObject* parent)
//    :QSortFilterProxyModel(parent) {

//    checks = new QStandardItemModel(this);

//}

//int DataProxy::rowCount(const QModelIndex& parent) const {
//    return QSortFilterProxyModel::rowCount(parent);
//}

//int DataProxy::columnCount(const QModelIndex& parent) const {
//    return QSortFilterProxyModel::columnCount(parent) + 1;
//}

////Qt::ItemFlags DataProxy::flags(const QModelIndex& index) const
////{
////    if (index.column() == DifferenceColumn)
////    {
////        return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
////    }
////    return QSortFilterProxyModel::flags(index);
////}

////Qt::ItemFlags DataProxy::flags(const QModelIndex& index) const {
////    Qt::ItemFlags result = this->sourceModel()->flags(index);
////    if (index.column() == 3)
////    {
////        result |= Qt::ItemIsEditable;
////    }
////    return result;
////}

//QVariant DataProxy::data(const QModelIndex& index, int role) const {

//    if (index.column() == checkColumn) {
//        return checks->data(checks->index(index.row(), checkColumn), role);
//    }

//    return QSortFilterProxyModel::data(mapToSource(index), role);

//}

//QVariant DataProxy::headerData(int section, Qt::Orientation orientation, int role) {

//    qDebug() << section << "\n";

//    if (orientation == Qt::Horizontal) {
//        if(section == checkColumn)
//            return "Selected";
//        else
//            return QSortFilterProxyModel::headerData(section-1, orientation, role);
//    }

//    return QSortFilterProxyModel::headerData(section, orientation, role);

//}

//void DataProxy::setSourceModel(QAbstractItemModel* sourceModel) {
//    QSortFilterProxyModel::setSourceModel(sourceModel);

//    int rows = sourceModel->rowCount();
//    qDebug() << rows << "\n";
//    for (int i = 0; i < rows; i++) {
//        QStandardItem* item = new QStandardItem();
//        item->setCheckable(true);
//        if (i % 2 == 0)
//            item->setCheckState(Qt::Unchecked);
//        else
//            item->setCheckState(Qt::Checked);
//        checks->setItem(i, 0, item);
//    }

//}

//QModelIndex DataProxy::mapFromSource(const QModelIndex& sourceIndex) const {

//    this->index(sourceIndex.row(), sourceIndex.column());

//}

//QModelIndex DataProxy::mapToSource(const QModelIndex& proxyIndex) const {
//    if (proxyIndex.column() == checkColumn)
//        return QModelIndex();
//    return sourceModel()->index(proxyIndex.row(), proxyIndex.column());
//}












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

QList<QStringList> ResultsList::getSelectedList() {

    QList<QStringList> ret;
    for (int r = 0; r < data->rowCount(); r++) {
        if (data->itemFromIndex(data->index(r, 0))->checkState() == Qt::Checked) {
            QStringList names;
            names.append(data->data(data->index(r, 1)).toString());
            names.append(data->data(data->index(r, 4)).toString());
            names.append(data->data(data->index(r, 2)).toString());
            ret.append(names);
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


//    this->setModel(data);

//    // old model
//    QItemSelectionModel *m = this->selectionModel();

//    QSqlTableModel* model = new QSqlTableModel(this, QSqlDatabase::database("database"));
//    model->setTable("logs");
//    model->setEditStrategy(QSqlTableModel::OnManualSubmit);
//    model->select();
//    model->setHeaderData(0, Qt::Horizontal, "Filename");
//    model->setHeaderData(1, Qt::Horizontal, "HasParams");
//    model->setHeaderData(2, Qt::Horizontal, "Comment");
//    model->setHeaderData(3, Qt::Horizontal, "ExportName");

//    DataProxy *proxyModel = new DataProxy(this);
//    proxyModel->setSourceModel(model);

//    this->setModel(proxyModel);

//    // delete old model
//    delete m;

//    for (int row = 0; row < 4; ++row) {
//        QStandardItem* item = new QStandardItem();
//        item->setCheckable(true);
//        item->setCheckState(Qt::Unchecked);
//        data->setItem(row, 0, item);
//        for (int column = 1; column < 5; ++column) {
//            QStandardItem *item = new QStandardItem(QString("row %0, column %1").arg(row).arg(column));
//            data->setItem(row, column, item);
//        }
//    }

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
        QStringList lline = line.split(";");
        for (int i = 0; i < lline.size(); i++) {
            QStandardItem* item;
            if (i != 1)
                item = new QStandardItem(lline[i]);
            else {
                if (lline[i] == "0")
                    item = new QStandardItem("NO");
                else
                    item = new QStandardItem("YES");
            }
            data->setItem(row, i+1, item);
        }
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
            if (c != 2)
                out << data->data(data->index(r, c)).toString() << ";";
            else {
                if (data->data(data->index(r, c)).toString() == "NO")
                    out << 0 << ";";
                else
                    out << 1 << ";";
            }
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
        QStringList lline = line.split(";");
        for (int i = 0; i < lline.size(); i++) {
            QStandardItem* item;
            if (i != 1)
                item = new QStandardItem(lline[i]);
            else {
                if (lline[i] == "0")
                    item = new QStandardItem("NO");
                else
                    item = new QStandardItem("YES");
            }
            data->setItem(row, i+1, item);
        }
        QStandardItem* item = new QStandardItem();
        item->setCheckable(true);
        item->setCheckState(Qt::Unchecked);
        data->setItem(row, 0, item);
        row++;
    }

    data->setHeaderData(0, Qt::Horizontal, "");
    data->setHeaderData(1, Qt::Horizontal, "Filename");
    data->setHeaderData(2, Qt::Horizontal, "HasParams");
    data->setHeaderData(3, Qt::Horizontal, "Comment");
    data->setHeaderData(4, Qt::Horizontal, "ExportName");

    file.close();

    data->resetChange();
    this->resizeColumnsToContents();

}

