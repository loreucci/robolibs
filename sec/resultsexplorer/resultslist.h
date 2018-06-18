#ifndef RESULTSLIST_H
#define RESULTSLIST_H

#include <QWidget>
#include <QTableView>
#include <QStandardItemModel>
#include <QTimer>
//#include <QSortFilterProxyModel>


//class DataProxy : public QSortFilterProxyModel {
//    Q_OBJECT

//public:
//    explicit DataProxy(QObject* parent = nullptr);

//    int rowCount(const QModelIndex& parent) const override;
//    int columnCount(const QModelIndex& parent) const override;

//    QVariant data(const QModelIndex &index, int role) const override;
//    QVariant headerData(int section, Qt::Orientation orientation, int role);

//    void setSourceModel(QAbstractItemModel *sourceModel) override;

//    virtual QModelIndex mapFromSource(const QModelIndex& sourceIndex) const override;

//    virtual QModelIndex mapToSource(const QModelIndex& proxyIndex) const override;

////    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;



//protected:
//    const int checkColumn = 0;

//    QStandardItemModel* checks;

//};


class FileModel : public QStandardItemModel {
    Q_OBJECT

public:
    explicit FileModel(QObject *parent = nullptr);

    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;

    bool isChanged();
    void resetChange();

public slots:
    void checkForChange(QStandardItem* item);

protected:
    bool changed;

};

struct ExportableEntry {

    unsigned int mode;
    QString basename;
    QString timestamp;
    QString trial;
    QString filelist;
    QString exportname;

};


class ResultsList : public QTableView {
    Q_OBJECT

public:
    explicit ResultsList(QWidget* parent = 0);
    void changeFile(QString datafile);

    bool isChanged();

    QList<ExportableEntry> getSelectedList();

public slots:
    void loadData();
    void addNewData();
    void saveChanges();
    void deleteSelected();
    void selectall();
    void deselectall();

protected:
    QString datafile;
    FileModel* data;
    QTimer* timer = nullptr;

    void loadFromFile();
    void parseLine(const QString& line, int row);

};

#endif // RESULTSLIST_H
