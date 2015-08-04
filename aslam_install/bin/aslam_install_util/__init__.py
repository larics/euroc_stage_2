from aslam_git import Repo, findGitsInDirectories
try:
    from termcolor import colored
except:
    print "Unable to import termcolor."
    print "Try:"
    print "sudo pip install termcolor"
    def colored(X,Y):
        return X

