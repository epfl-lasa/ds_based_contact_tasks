Host *
	User	git
	IdentityFile	~/.ssh/id_ci_rsa
	StrictHostKeyChecking	no
	PasswordAuthentication	no
	CheckHostIP	no
	BatchMode	yes