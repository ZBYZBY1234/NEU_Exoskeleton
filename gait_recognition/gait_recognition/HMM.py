# Hidden Markov Model
# mode: (*)Stay,(*)Walk,(*)Stand Up,(*)Squat down,(*)Run
class HMM
# Main Function
if __name__ == "__main__":
    # nn = Network(3,3,3)
    # cases  = [np.array([1,0,0]),np.array([0,1,0]),np.array([0,0,1])]
    # labels = [np.array([0,1,0]),np.array([0,0,1]),np.array([1,0,0])]
    # nn.train(case,label)
    nn = Network(10,10,3)

    cases,labels = pre_labels_cases()
    nn.train(cases,labels,limit=500,learn=0.1)
    print(nn.input_weights)
    print(nn.output_weights)
    for i in range(len(cases)):
        print(nn.predict(cases[i]),labels[i])