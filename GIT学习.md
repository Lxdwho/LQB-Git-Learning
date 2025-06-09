#  GIT学习

首先init一下仓库

```shell
git init
```

随后git add文件将文件添加到仓库

```shell
git add . # .表示当前路径下的所有文件
```

通过git status可以查看状态

随后通过git commit 将代码提交到仓库

```shell
git commit -m "代码提交的注释"
# 也可以不加-m
```

第一次添加的话需要创建key

```shell
ssh-keygen -t rsa -C "youremail@example.com"
```

随后可以在用户下的.ssh目录里找到id_rsa和id_rsa.pub这两个文件，复制id_rsa.pub这个文件的内容到github中添加ssh密钥就可以

再接下来在github中创建项目，拿到项目地址比如：https://github.com/yourGitHubName/projectName.git

再将本地仓库与项目链接

```shell
git remote add origin https://github.com/yourGitHubName/projectName.git
```

最后推送仓库

- 当你第一次推送本地分支到远程仓库时，使用 `-u` 选项可以告诉 Git 将当前本地分支与远程分支建立关联关系。这意味着，Git 会记住这个关联，以后你可以直接使用 `git push` 和 `git pull` 而不需要每次都指定远程仓库和分支名称。

```shell
git push -u origin main # 第一次可能要加-u，-u表示设置上游分支（Upstream Branch）
git push origin main
```

当你有一些不想上传的文件时，可以创建.gitignore文件，在文件中加入不上传的文件夹路径即可`.gitignore`

后面又有想要上传的文件时只需要在提交到仓库后进行推送即可