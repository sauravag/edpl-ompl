%-----------------------------------------------------------------------------
% UQ_THESIS.STY	Primary style for U.Q. Ph.D. Thesis by
%               Chris S. Craddock
%               Modified version of cit_thesis.sty by
%               Michael H. Kelsey
%
% USAGE:  (LaTeX 2.09) \documentstyle[draftfoot,preprint]{cit_thesis}
%         (LaTeX2e)    \documentclass[draftfoot,preprint]{cit_thesis}
%
% Modifies the {report} style, so that the title page and chapters
% are handled according to the Caltech Regulations for Format and
% Presentation of Theses (Office of Graduate Studies), and K. Turabian,
% _A Manual for Writers_ (U. Chicago Press, Chicago, 1987).
%-----------------------------------------------------------------------------
% The secondary style of [preprint] sets the conditional ``\ifpreprintsty''
% TRUE.  The command ``\preprint'', at the beginning of the document, has
% the same effect.  This causes a local publication number (see ``\pubnum''
% below) to be typeset in the upper right corner of the title page.  The author
% may also write conditional macros using the ``\ifpreprintsty''
% construction.
%
% For preliminary or working copies of the thesis, the secondary style of
% [draftfoot] may be used, or the command ``\draftfoot'' may be specified at
% the beginning of the document.  This produces a line at the bottom of every
% page, with ``DRAFT'' in the lower left corner, and the date and time when
% LaTeX was executed in the lower right.
% 
% A number of useful macros are defined for use in this style.  The
% title page of a thesis contains several standard phrases with possibly
% ``variable'' information, such as the degree to be awarded, the name
% and address of the institution, and copyright information.  The macros
% below are used to specify these data; the macro arguments indicate
% the default values as defined in this style file.
%
% \degreeaward{Doctor of Philosophy}			Degree to be awarded
% \university{University of Queensland}			Institution name
% \address{Brisbane, Australia}				Institution address
% \unilogo{cit_logo}                                    Institution logo
% \copyyear{\the\year}					Year of diploma award
%
% \pubnum{}					     Publication/preprint number
%
% The usual LaTeX macros ``\author,'' ``\title,'' and ``\date'' work as
% defined in the LaTeX manual.  The argument to \date should be the date
% on which the candidate's final examination (defense) is scheduled.
%
% The publication/preprint number will be typeset in the upper right corner
% of the title page if and only if the [preprint] style is selected, or if
% the ``\preprint'' command was executed before \maketitle.
%
% The command ``\maketitle'' will generate a title page formatted according
% to the Caltech instructions, and including all of the information defined
% by the macros above.  Unlike the standard version, ``\maketitle'' may be
% repeated several times in a document:  the terms defined for it are _not_
% reset after the page is formatted.
%
% The command ``\extrachapter'' will generate an unnumbered chapter heading,
% similar to the LaTeX ``\chapter*'' command.  In addition, ``\extrachapter''
% will produce an entry in the Table of Contents.  This command is used in
% the {abstract} and {acknowledgements} environments, listed below.
%
% The initial portion of the thesis document, typically the copyright page,
% abstract, and tables of contents, figures, and tables, are numbered in
% roman numerals, while the body of the thesis is numbered in arabic.  The
% {frontmatter} environment handles these page numbering conventions.  The
% author should place
%
%	\begin{frontmatter}
%	...(abstract, contents, etc.)
%	\end{frontmatter}
%
% immediately following the \maketitle command.  The first entry in the
% {frontmatter} environment should generally be ``\makecopyright.''  This
% command generates a spearate copyright page, with the text
%
%				 (C) \copyyear
%				    \author
%			      All Rights Reserved
%
% centered at the bottom of the page.  Obviously, the items indicated with
% macro commands above are filled in with the actual arguments to those
% commands.
%
% The abstract is produced with the {abstract} environment, which generates
% an unnumbered chapter with an entry in the table of contents.  The author
% should use
%
%	\begin{abstract}
%	...(body of abstract goes here)
%	\end{abstract}
%
% within the {frontmatter} environment.
%
% An acknowledgements page is produced in the same way as the abstract, with
% the {acknowledgements} environment.
%
%
% 950201  Michael H. Kelsey    Adapted from original of unknown source.
% 950208  -- Merged with "citthesis.cls" from Blair Zajac.
%         Change [draft] and \draft to ``draftfoot''.
% 950525  -- Per request from B. McKenney, chapter headings are done in
%         \LARGE font, not \Large.
% 960319  -- Force [letterpaper] option to {report} class
% 960517  -- Allow double-sided generation (remove \@twosidedfalse)
%         -- Change \ifpreprintstyle to \ifpreprintsty for REVTeX
%            compatibility
% 960521  -- Fix \evensidemargin setting for double-ssided support
%-----------------------------------------------------------------------------
%%%
%%% Pre-define commands required for option-processing
%%%
%
% \draftfoot will put "DRAFT" and the current date and time in the footer
%
\newif\ifthesisdraft
\def\draftfoot{\thesisdrafttrue\thesisfooter{Draft Copy\hss\thesistime}}

\newif\if@appmatter \@appmatterfalse

%
% \preprint will set the conditional \ifpreprintstyle to be TRUE
%
\newif\ifpreprintsty
\def\preprint{\preprintstytrue}
%
% for MiKTeX use
%\newfont{\enormoustimes}{ptmr8t scaled 3500}
%\newfont{\enormoustimesbold}{ptmb8t scaled 3500}
% else
\newfont{\enormoustimes}{ptmr scaled 3500}
\newfont{\enormoustimesbold}{ptmb scaled 3500}
\newcommand{\HRule}{\rule{\linewidth}{0.9 mm}}
%
%
%%%
%%% Select between LaTeX 2.09 and LaTeX2e, based on \documentclass command
%%%
%
\ifx\undefined\documentclass		%%% LaTeX 2.09 configuration %%%
%
  \typeout{Document style `uq_thesis'.  2 June 1998, M. Kelsey nad C. Craddock}
%
% Define actions for style options
%
  \@namedef{ds@draftfoot}{\thesisdrafttrue}
  \@namedef{ds@preprint}{\preprintstytrue}
%
%  \input report.sty\relax		%% Standard REPORT format
  \input ./styles/report_1.sty\relax
  \input setspace.sty\relax		%% Load package for double-spacing
  \input ./styles/fancyheadings.sty\relax
  \input ./styles/caption2.sty\relax
%  \input ./styles/chicago.sty\relax
%
\else					%%% LaTeX2e configuration %%%
  \NeedsTeXFormat{LaTeX2e}[1994/06/01]
  \ProvidesClass{./styles/uq_thesis}[1998/02/06 Uq Thesis class]
%
% Define actions for style options
%
  \DeclareOption{draftfoot}{\thesisdrafttrue}
  \DeclareOption{preprint}{\preprintstytrue}
%
%  \DeclareOption*{\PassOptionsToClass{\CurrentOption}{report}}
  \DeclareOption*{\PassOptionsToClass{\CurrentOption}{./styles/report_1}}
  \ProcessOptions*			%% Process remaining options locally
  \LoadClass[a4paper]{./styles/report_1}		%% Standard REPORT format
  \RequirePackage{setspace}		%% Load package for double-spacing
  \RequirePackage{./styles/fancyheadings}
  \RequirePackage{./styles/caption2}
%  \RequirePackage{./styles/chicago}
\fi
%
%%%
%%% Override standard REPORT format for thesis layout requirements
%%%
%
% Define \thispagestyle to null so that \chapter doesn't change page style.
%
%\let\thesis@document\document
%\def\document{\thesis@document\thispagestyle{empty}\def\thispagestyle##1{}}
%%
% Define commands to generate draft-mode footers
%
\def\thesisfooter#1{\def\@oddfoot{#1} \def\@evenfoot{#1}}
%
% Construct timestamp information for draft-mode footer
%
\newcount\timehh\newcount\timemm
\def\thesistime{%
  \timehh=\time\divide\timehh by 60%
  \timemm=\timehh\multiply\timemm by -60%
  \advance\timemm by \time%
  \ifnum\timehh<10 0\fi\number\timehh\,:\,%
  \ifnum\timemm<10 0\fi\number\timemm%
  \ \ \today%
}				%% End of \thesistime
%
%\setstretch{1.5}			%% ``Double-spaced'' text
%\setstretch{1.25}                        %% ``1.5-spaced'' text

% lengths for letterpaper
%\setlength{\oddsidemargin}{0.66cm}	%% UQ binding margin ( 30mm )
%\setlength{\evensidemargin}{0.66cm}	%% 
%\setlength{\textwidth}{15.0cm}		%% Makes right margin = 1 inch
%\setlength{\topmargin}{-0.75cm}		%% Top margin = 3/4'' inch (incl pg#)
%\setlength{\headsep}{.15in}		%% Space between page # and text
%\setlength{\footskip}{.4in}		%% Space from bottom of text to foot
%\setlength{\headheight}{14.5pt}
%\setlength{\textheight}{24.6cm}        %% Makes bottom margin = 1 inch

% lengths for a4paper, Diploma
%\setlength{\textheight}{239mm}
%\setlength{\textwidth}{164.1mm}
%\setlength{\voffset}{-20mm}
%\setlength{\hoffset}{-28mm }
%\setlength{\oddsidemargin}{25mm}
%\setlength{\evensidemargin}{25mm}
%\setlength{\topmargin}{14mm}
%\setlength{\headheight}{5.5mm} % 5
%\setlength{\headsep}{7,5mm}
%\setlength{\parindent}{0mm}
% lengths for a4paper, PhD
\setlength{\textheight}{225mm}
\setlength{\textwidth}{145mm}
\setlength{\voffset}{-1in} % default margins are 1in=254mm
\setlength{\hoffset}{-1in}
\setlength{\oddsidemargin}{40mm}
\setlength{\evensidemargin}{40mm}
\setlength{\topmargin}{30mm}
\setlength{\headheight}{5mm}
\setlength{\headsep}{5mm}
\setlength{\topskip}{2mm}
\setlength{\footskip}{0mm}
\setlength{\parindent}{0mm}
\setlength{\parskip}{5pt plus 1pt}
%\setlength{\parskip}{9pt plus 1pt}  %% ``Double-spaced'' text
%\setlength{\parskip}{7pt plus 1pt}  %% ``1.5-spaced'' text

%\flushbottom
%
% Set the headers and footers. Page number to the top outside, empty footer.
%
\pagestyle{fancyplain}
\renewcommand{\chaptermark}[1]{\markboth{#1}{#1}}
\renewcommand{\sectionmark}[1]{\markright{\thesection\ #1}}

\let\@oddfoot\@empty \let\@evenfoot\@empty

\ifthesisdraft\draftfoot\fi

%
% These are the commands from caption2.sty
%
\captionstyle{normal}
\renewcommand{\captionfont}{\small}
\renewcommand{\captionlabelfont}{\small \bf}

%
% Define spacing for use on titlepage
%
\def\titleskip{\vskip 4\bigskipamount}		%% Whitespace after title
\def\authorskip{\vskip 2\bigskipamount}		%% Whitespace after author
%
% Create a command for unnumbered chapters, with a Table of Contents
% entry.
%
\def\extrachapter{\@ifnextchar[{\@lextrachapter}{\@extrachapter}}

\def\@lextrachapter[#1]#2{%	User specified optional contents entry
  \chapter*{#2}\addcontentsline{toc}{chapter}{#1}%
}

\def\@extrachapter#1{%		Chapter heading used as contents entry
  \chapter*{#1}\addcontentsline{toc}{chapter}{#1}%
}
%
% Chapter headings should be at top of page, single line format
% with single font size of \Large.
%
%\def\@makechapterhead#1{\vspace*{10pt}
%  {\parindent 0pt \raggedright \LARGE\bf%
%   \ifnum \c@secnumdepth>\m@ne%
%     \@chapapp{} \thechapter \hskip 2.3ex plus .2ex
%   \fi
%   #1\par \nobreak \vskip 20pt}
%}				%% End of \@makechapterhead
\def\@makechapterhead#1{%
  {\parindent \z@ \raggedleft \normalfont
    \ifnum \c@secnumdepth >\m@ne
%      \if@mainmatter
        \large \bfseries
        \if@appmatter
        A~P~P~E~N~D~I~X \hspace{0.5cm}
        \else
       C~H~A~P~T~E~R \hspace{0.5cm}
        \fi
        \baselineskip 30pt
        \space \enormoustimesbold \thechapter
        \par\nobreak
        \HRule
        \par\nobreak
        \vskip 5\p@
%      \fi
    \fi
    \interlinepenalty\@M
    \LARGE #1\par\nobreak
    \HRule
    \par\nobreak
    \vskip 40\p@
  }}

%\def\@makeschapterhead#1{\vspace*{10pt}
\def\@makeschapterhead#1{  
    {\parindent \z@ \raggedleft \normalfont
        \large \bfseries
%        B~I~B~L~I~O~G~R~A~P~H~Y \hspace{0.5cm}
%        \baselineskip 30pt
%        \space \enormoustimesbold \thechapter
%        \par\nobreak
        \HRule
        \par\nobreak
        \vskip 5\p@
    \interlinepenalty\@M
    \LARGE #1\par\nobreak
    \HRule
    \par\nobreak
    \vskip 40\p@

%  {\parindent 0pt \raggedright \LARGE\bf%
%   #1\par \nobreak \vskip 20pt}
}}				%% End of \@makeschapterhead

%
% We want roman page numbers in the frontmatter, and arabic afterward.
% use \begin{frontmatter} ... \end{frontmatter} to do this.
%
\newenvironment{frontmatter}%
{\pagenumbering{roman}
 \pagestyle{fancyplain}\headrulewidth0pt
 \lhead[\fancyplain{}{\bfseries \thepage}]
       {\fancyplain{}{}}
 \rhead[\fancyplain{}{}]
       {\fancyplain{}{\bfseries \thepage}}
 \cfoot{}}
{\clearpage\pagenumbering{arabic}
 \lhead[\fancyplain{}{\bfseries \thepage}]
       {\fancyplain{}{\bfseries \rightmark}}
 \rhead[\fancyplain{}{\bfseries \leftmark}]
       {\fancyplain{}{\bfseries \thepage}}}

\newenvironment{appmatter}%
  {
  \appendix
  \renewcommand{\thepage}{\Alph{chapter}-\arabic{page}} 
  \@appmattertrue
  }

%
% Additional titlepage definitions
%
\def\degreeaward{\gdef\@degreeaward}
\degreeaward{Ph.D} % Default is Ph.D.

\def\university{\gdef\@university}
\university{Texas A&M University}			

\def\unilogo{\gdef\@unilogo}
\unilogo{styles/tamu-logo}                                  

\def\address{\gdef\@address}
\address{College Station, Texas, USA}				

\def\copyyear{\gdef\@copyyear}
\copyyear{\the\year}					% Default is today

\def\pubnum{\gdef\@pubnum}
\pubnum{}						% No publication number
%
% The titlepage must conform to CIT requirements
%
\def\maketitle{%
 {\parindent=0pt
 \begin{titlepage}
   \let\footnotesize\small \let\footnoterule\relax \setcounter{page}{1}
   \vskip-\headsep
   \ifpreprintsty\ifx\empty\@pubnum\else
      {\singlespace\hfill\hbox{\begin{tabular}{l}\@pubnum\end{tabular}}}
      \titleskip
   \fi\fi
   \begin{center}
     {\Large\bf \@title\par}
     \titleskip
     \textit{by} \\
     {\large \@author\par}
     \authorskip
     %Submitted in partial fulfillment for the degree of
     %\@degreeaward\par
     \vfill
     \includegraphics[width=4cm]{./styles/cranfield_crest.jpg} \\
    \vspace{1.2cm}
     \@university\\ 
     \@address\par
     \authorskip
     \@copyyear\par
%     {\normalsize (Submitted \@date)\par}
   \end{center}
   \vspace*{.5in}
 \end{titlepage}}
}				%% End of \maketitle
%
% The copyright page is new, must conform to CIT requirements
%
\def\makecopyright{%
 \setcounter{page}{2}
 {\parindent=0pt
 \null				% Forces rest to bottom of page
 \vfill
 \begin{center}
   \copyright\quad \@copyyear\\
   \@author\\
   All Rights Reserved
 \end{center}}
 \newpage
}				%% End of \makecopyright
%
% The abstract and acknowledgements are treated as unnumbered chapters
% with Table of Contents entries.
%
\def\abstract{\extrachapter{Abstract}}		% \begin{abstract}
\def\endabstract{\clearpage}			% \end{abstract}

\def\acknowledgements{\extrachapter{Acknowledgements}}
\def\endacknowledgements{\clearpage}

\def\statementoforig{\extrachapter{Statement of Originality}}
\def\endstatementoforig{\clearpage}

\def\nomenclature{\extrachapter{Nomenclature}}
\def\endnomenclature{\clearpage}

\def\lop{\extrachapter{List of Publications}}
\def\endlop{\clearpage}

%
% The {thebibliography} environment should put an entry into the Table
% of Contents, just as the {abstract} and {acknowledgements} environments
% do.
%
\let\thesis@thebibliography=\thebibliography

\def\thebibliography#1{\thesis@thebibliography{#1}%
  \addcontentsline{toc}{chapter}{Bibliography}}

\bibliographystyle{plain}
%\bibliographystyle{./styles/uq_bib}
%
\endinput

