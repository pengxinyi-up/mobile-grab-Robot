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

#ifndef VOCABULARY_H_
#define VOCABULARY_H_

#include <QtCore/QMultiMap>
#include <QtCore/QVector>
#include <opencv2/opencv.hpp>

namespace find_object {

class Vocabulary {
public:
	Vocabulary();
	virtual ~Vocabulary();

	void clear();
	QMultiMap<int, int> addWords(const cv::Mat & descriptors, int objectId);
	void update();
	void search(const cv::Mat & descriptors, cv::Mat & results, cv::Mat & dists, int k);
	int size() const {return indexedDescriptors_.rows + notIndexedDescriptors_.rows;}
	int dim() const {return !indexedDescriptors_.empty()?indexedDescriptors_.cols:notIndexedDescriptors_.cols;}
	int type() const {return !indexedDescriptors_.empty()?indexedDescriptors_.type():notIndexedDescriptors_.type();}
	const QMultiMap<int, int> & wordToObjects() const {return wordToObjects_;}
	const cv::Mat & indexedDescriptors() const {return indexedDescriptors_;}

	void save(QDataStream & streamSessionPtr, bool saveVocabularyOnly = false) const;
	void load(QDataStream & streamSessionPtr, bool loadVocabularyOnly = false);
	bool save(const QString & filename) const;
	bool load(const QString & filename);

private:
	cv::flann::Index flannIndex_;
	cv::Mat indexedDescriptors_;
	cv::Mat notIndexedDescriptors_;
	QMultiMap<int, int> wordToObjects_; // <wordId, ObjectId>
	QVector<int> notIndexedWordIds_;
};

} // namespace find_object

#endif /* VOCABULARY_H_ */
