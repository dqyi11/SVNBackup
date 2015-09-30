'''
Created on 2013-5-29

@author: Walter
'''

if __name__ == '__main__':
    
    from matplotlib import pyplot as plt
    import matplotlib

    fracs = [0.5, 0.5]
    labels = ['True', 'False']
    explode=(0.05, 0)

    fig = plt.figure(1, figsize=(6,6)) 
    ax = plt.axes([0.1, 0.1, 0.8, 0.8])
    plt.title("Prior of P(Burglary | JohnCalls=T, MaryCalls=T)")
    def autopct_func():
        return '%1.1f%%'
    res = ax.pie(fracs,
                 labels=labels,
                 explode=explode,
                 autopct=autopct_func(),
                 shadow=True, 
                 startangle=90
                )
    
    plt.show()