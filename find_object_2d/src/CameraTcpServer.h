/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CAMERATCPCLIENT_H_
#define CAMERATCPCLIENT_H_

#include <QtGlobal>
#include <QtNetwork/QTcpServer>
#include <opencv2/opencv.hpp>

namespace find_object {

class CameraTcpServer : public QTcpServer
{
	Q_OBJECT;
public:
	CameraTcpServer(quint16 port = 0, QObject * parent = 0);
	cv::Mat getImage();
	int imagesBuffered() const {return images_.size();}
	bool isConnected() const;

	QHostAddress getHostAddress() const;
	quint16 getPort() const;

protected:
#if QT_VERSION >= 0x050000
	virtual void incomingConnection ( qintptr socketDescriptor );
#else
	virtual void incomingConnection ( int socketDescriptor );
#endif

private Q_SLOTS:
	void readReceivedData();
	void displayError(QAbstractSocket::SocketError socketError);
	void connectionLost();

private:
	quint64 blockSize_;
	QVector<cv::Mat> images_;
};

} // namespace find_object

#endif /* CAMERATCPCLIENT_H_ */
