#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QTextEdit>
#include "passthroughfiltercpp.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
          ui->setupUi(this);

          QPushButton *btnGetFile = new QPushButton(this);
          txt = new QTextEdit();

          btnGetFile->setText("Select File");
          btnGetFile->setFixedWidth(100);
          btnGetFile->setFixedHeight(30);
          txt->setFixedHeight(30);

          QObject::connect(btnGetFile, SIGNAL(clicked()),this, SLOT(clickedSlot()));
          btnGetFile->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);

          QWidget* centralWidget = new QWidget(this);
          centralWidget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);

          QVBoxLayout* layout = new QVBoxLayout(centralWidget);
          QHBoxLayout* hLayout = new QHBoxLayout(centralWidget);
          hLayout->addWidget(txt);
          hLayout->addWidget(btnGetFile);
          layout->addLayout(hLayout);

          setCentralWidget(centralWidget);
          setWindowTitle("Spatial Partitioning");
}

void MainWindow::clickedSlot()
{
    QString fileName = QFileDialog::getOpenFileName(this,
    tr("Select File"), "/home/", tr("Files (*.*)"));
    txt->setText(fileName);

    PassThroughFiltercpp* obj = new PassThroughFiltercpp();
    obj->ApplyFilter(fileName.toUtf8().constData());
    //obj->SearchCircle(fileName.toUtf8().constData(), 0.5f);
    //obj->SearchVoxel(fileName.toUtf8().constData(), 0.5f);
    //obj->SearchKNearest(fileName.toUtf8().constData(), 0.5f, 2000);
}

MainWindow::~MainWindow()
{
    delete ui;
}
