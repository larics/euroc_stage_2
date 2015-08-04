import os

try:
    import git
except:
    print "Unable to import the git python module."
    print "Try:"
    print "  sudo apt-get install python-git"
    print "or for OSX:"
    print "  sudo pip install GitPython"
    exit

try:
    from termcolor import colored
except:
    print "Unable to import termcolor."
    print "Try:"
    print "sudo pip install termcolor"
    def colored(X,Y):
        return X

class Repo(object):
    def __init__(self, name, directory):
        self.name = name
        self.repo = git.Repo(directory)
    def getUntrackedFiles(self, ignoreDirectories = True):
        return self.repo.git.ls_files('--exclude-standard','--others', '--directory' if not ignoreDirectories else None).split()
    
    def __upstreamProblems(self, contextHelp):
        print colored( "Error: unable to query the upstream branch for repository {0}".format(self.name) , "red")
        if(contextHelp):
            print """To create a new branch that tracks an existing remote branch, try the git command:
    
      git checkout --track origin/remote_branch_name
    
    To push the current branch to the remote repository, try:
    
      git push origin local_branch_name
    
    To set the current branch to track an existing remote branch, try:
    
      git branch --set-upstream local_branch_name origin/remote_branch_name"""
        else:
            print colored("(Use -H to get context help.)", "red")

    def getCommits(self, selector) :
        if hasattr(self.repo.__class__, 'log') :
            return self.repo.log(selector)
        elif hasattr(self.repo.__class__, 'commits') :
            return self.repo.commits(selector)
        else :
            return list(self.repo.iter_commits(selector))
        
    def getUnpushedCommits(self, contextHelp = False):
        try:
            return self.getCommits('@{upstream}..')
        except Exception as ex:
            print ex
            self.__upstreamProblems(contextHelp)
            return []

    def getUnmergedCommits(self, contextHelp = False):
        try:
            return self.getCommits('..@{upstream}')
        except:
            return []
        
    def getStashed(self):
        return [ line for line in self.repo.git.stash('list').split("\n") if line];

    def isDirty(self):
        return self.repo.is_dirty() if callable(self.repo.is_dirty) else self.repo.is_dirty
    def status(self):
        return self.repo.git.status()
    def fetch(self):
        return self.repo.git.fetch()
    def fetchAll(self):
        return self.repo.git.fetch('--all')
    def fastforward(self):
        return self.repo.git.merge("--ff-only")
    def merge(self):
        return self.repo.git.merge()
    def pull(self):
        return self.repo.git.pull()

    def hexSha(self):
        if hasattr(self.repo.__class__, 'head') : return self.repo.head.commit.hexsha;
        else : return self.repo.git.show_ref('-s', '--head').split('\n')[0];
    
    def activeBranch(self):
        try:
            return self.repo.active_branch
        except:
            return "NONE"
    def trackedBranch(self):
        active = self.activeBranch()
        if(active):
            try:
               if hasattr(active.__class__, 'tracking_branch'):
                   return active.tracking_branch()
               else:
                   return self.repo.git.for_each_ref('--format=%(upstream:short)', ('refs/heads/' + active))
            except:
                return ""
        else:
            return ""


def findGitsInDirectories(directories, maxdepth = 1, filters = [], nested = False):
    submodules = {}
    __findGitsInDirectories(directories, submodules, maxdepth, filters, nested);
    return submodules;

def __findGitsInDirectories(directories, submodules, maxdepth, filters, nested):
    for dir in directories:
        if(os.path.isdir(dir)):
            ignore = isWC = False;
            name = os.path.basename(dir);
            for filter in filters:
                if (not filter.match(name, dir)) : ignore = True; break; 
            if(not ignore):
                pathGit = os.path.join(dir, ".git");
                if(os.path.isdir(pathGit)) :
                    submodules[name] = dir;
                    isWC = True;
            if(maxdepth and (nested or not isWC)):
                __findGitsInDirectories([ os.path.join(dir, file) for file in os.listdir(dir) ], submodules, maxdepth - 1, filters, nested);
