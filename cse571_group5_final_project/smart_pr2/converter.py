import pickle

with open("ObjectClassifier.pkl","rb") as fp:
	w = pickle.load(fp)

pickle.dump(w, open("ObjectClassifier.pkl","wb"), protocol=2)