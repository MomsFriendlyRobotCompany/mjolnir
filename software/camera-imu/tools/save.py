import pickle


def to_pickle(data, filename):
    with open(filename, 'wb') as fd:
        d = pickle.dumps(data)
        fd.write(d)
