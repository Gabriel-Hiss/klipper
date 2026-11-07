//! Port of the `shlex` subset used by `klippy/gcode.py`.
//!
//! The reference is CPython 3.8 `Lib/shlex.py`. Klippy builds the lexer as
//! `shlex.shlex(rawparams, posix=True)` and then sets
//! `whitespace_split = True` and `commenters = '#;'` (see
//! `GCode._get_extended_params`). The `ValueError` texts are `No closing
//! quotation` and `No escaped character`, raised in the quote and escape
//! states of `read_token`.
//!
//! `punctuation_chars` is never set by klippy, so the `'c'` punctuation
//! state is unreachable here; the word-character and whitespace rules keep
//! the `posix` additions of `shlex.__init__`.

use core::fmt;

/// Error mirror of the `ValueError` raised by `shlex.shlex`.
#[derive(Debug, Clone, PartialEq)]
pub enum ShlexError {
    /// `ValueError("No closing quotation")`.
    NoClosingQuotation,
    /// `ValueError("No escaped character")`.
    NoEscapedCharacter,
}

impl fmt::Display for ShlexError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoClosingQuotation => write!(f, "No closing quotation"),
            Self::NoEscapedCharacter => write!(f, "No escaped character"),
        }
    }
}

impl std::error::Error for ShlexError {}

#[allow(dead_code)]
#[derive(Debug, Clone, PartialEq)]
enum State {
    /// `' '` whitespace / start of token.
    Whitespace,
    /// `'a'` word.
    Word,
    /// A closing quote character.
    Quoted(char),
    /// A backslash escape character.
    Escaped(char),
    /// `'c'` punctuation, unreachable but kept for state parity.
    Punctuation,
    /// `None`: end of file.
    Eof,
}

/// Lexer mirroring `shlex.shlex(instream, posix=True)` after klippy sets
/// `whitespace_split` and `commenters`.
pub struct Shlex {
    chars: Vec<char>,
    pos: usize,
    state: State,
    token: String,
    commenters: Vec<char>,
    whitespace_split: bool,
    posix: bool,
    wordchars: Vec<char>,
    whitespace: Vec<char>,
    quotes: Vec<char>,
    escape: Vec<char>,
    escapedquotes: Vec<char>,
    lineno: usize,
}

impl Shlex {
    /// `shlex.shlex(s, posix=True)` with `whitespace_split=True` and
    /// `commenters='#;'`, exactly as `klippy/gcode.py` configures it.
    pub fn new_klippy(input: &str) -> Self {
        let mut lex = Self::new(input, true);
        lex.whitespace_split = true;
        lex.commenters = vec!['#', ';'];
        lex
    }

    /// `shlex.shlex(s, posix)`.
    pub fn new(input: &str, posix: bool) -> Self {
        let mut wordchars: Vec<char> = (b'a'..=b'z').chain(b'A'..=b'Z').chain(b'0'..=b'9').map(|b| b as char).chain(['_']).collect();
        if posix {
            for c in "ßàáâãäåæçèéêëìíîïðñòóôõöøùúûüýþÿ".chars() {
                if !wordchars.contains(&c) {
                    wordchars.push(c);
                }
            }
            for c in "ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖØÙÚÛÜÝÞ".chars() {
                if !wordchars.contains(&c) {
                    wordchars.push(c);
                }
            }
        }
        Self {
            chars: input.chars().collect(),
            pos: 0,
            state: State::Whitespace,
            token: String::new(),
            commenters: vec!['#'],
            whitespace_split: false,
            posix,
            wordchars,
            whitespace: vec![' ', '\t', '\r', '\n'],
            quotes: vec!['\'', '"'],
            escape: vec!['\\'],
            escapedquotes: vec!['"'],
            lineno: 1,
        }
    }

    fn read_char(&mut self) -> Option<char> {
        if self.pos < self.chars.len() {
            let c = self.chars[self.pos];
            self.pos += 1;
            Some(c)
        } else {
            None
        }
    }

    fn consume_comment(&mut self) {
        // `readline()` in CPython consumes through the next `\n`.
        while let Some(c) = self.read_char() {
            if c == '\n' {
                self.lineno += 1;
                break;
            }
        }
        // EOF without newline also ends the comment.
        if self.pos >= self.chars.len() {
            // Nothing more to read.
        }
    }

    /// `read_token`: emits one token or `None` at EOF.
    fn read_token(&mut self) -> Result<Option<String>, ShlexError> {
        let mut quoted = false;
        let mut escapedstate = State::Whitespace;
        loop {
            let nextchar = self.read_char();
            if nextchar == Some('\n') {
                self.lineno += 1;
            }
            match self.state {
                State::Eof => {
                    self.token.clear();
                    break;
                }
                State::Whitespace => {
                    let Some(c) = nextchar else {
                        self.state = State::Eof;
                        break;
                    };
                    if self.whitespace.contains(&c) {
                        if !self.token.is_empty() || (self.posix && quoted) {
                            break;
                        }
                        continue;
                    } else if self.commenters.contains(&c) {
                        self.consume_comment();
                        continue;
                    } else if self.posix && self.escape.contains(&c) {
                        escapedstate = State::Word;
                        self.state = State::Escaped(c);
                    } else if self.wordchars.contains(&c) {
                        self.token.push(c);
                        self.state = State::Word;
                    } else if self.quotes.contains(&c) {
                        if !self.posix {
                            self.token.push(c);
                        }
                        self.state = State::Quoted(c);
                    } else if self.whitespace_split {
                        self.token.push(c);
                        self.state = State::Word;
                    } else {
                        self.token.push(c);
                        if !self.token.is_empty() || (self.posix && quoted) {
                            break;
                        }
                        continue;
                    }
                }
                State::Quoted(q) => {
                    quoted = true;
                    let Some(c) = nextchar else {
                        return Err(ShlexError::NoClosingQuotation);
                    };
                    if c == q {
                        if !self.posix {
                            self.token.push(c);
                            self.state = State::Whitespace;
                            break;
                        }
                        self.state = State::Word;
                    } else if self.posix && self.escape.contains(&c) && self.escapedquotes.contains(&q) {
                        escapedstate = State::Quoted(q);
                        self.state = State::Escaped(c);
                    } else {
                        self.token.push(c);
                    }
                }
                State::Escaped(esc) => {
                    let Some(c) = nextchar else {
                        return Err(ShlexError::NoEscapedCharacter);
                    };
                    let in_quotes = matches!(escapedstate, State::Quoted(q) if self.quotes.contains(&q));
                    let quote_char = match escapedstate {
                        State::Quoted(q) => Some(q),
                        _ => None,
                    };
                    if in_quotes {
                        if c != esc && Some(c) != quote_char {
                            self.token.push(esc);
                        }
                    }
                    self.token.push(c);
                    self.state = escapedstate.clone();
                    let _ = escapedstate;
                }
                State::Word => {
                    let Some(c) = nextchar else {
                        self.state = State::Eof;
                        break;
                    };
                    if self.whitespace.contains(&c) {
                        self.state = State::Whitespace;
                        if !self.token.is_empty() || (self.posix && quoted) {
                            break;
                        }
                        continue;
                    } else if self.commenters.contains(&c) {
                        self.consume_comment();
                        if self.posix {
                            self.state = State::Whitespace;
                            if !self.token.is_empty() || (self.posix && quoted) {
                                break;
                            }
                            continue;
                        }
                        continue;
                    } else if self.posix && self.quotes.contains(&c) {
                        self.state = State::Quoted(c);
                    } else if self.posix && self.escape.contains(&c) {
                        escapedstate = State::Word;
                        self.state = State::Escaped(c);
                    } else if self.wordchars.contains(&c)
                        || self.quotes.contains(&c)
                        || self.whitespace_split
                    {
                        self.token.push(c);
                    } else {
                        self.state = State::Whitespace;
                        if !self.token.is_empty() || (self.posix && quoted) {
                            break;
                        }
                        continue;
                    }
                }
                State::Punctuation => {
                    // Unreachable with empty `punctuation_chars`.
                    self.state = State::Whitespace;
                    break;
                }
            }
        }
        let result = core::mem::take(&mut self.token);
        if self.posix && !quoted && result.is_empty() {
            Ok(None)
        } else {
            Ok(Some(result))
        }
    }

    /// Drains the lexer, mirroring `list(lex)`.
    pub fn tokens(&mut self) -> Result<Vec<String>, ShlexError> {
        let mut out = Vec::new();
        while let Some(tok) = self.read_token()? {
            out.push(tok);
        }
        Ok(out)
    }
}

impl Iterator for Shlex {
    type Item = Result<String, ShlexError>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.read_token() {
            Ok(Some(tok)) => Some(Ok(tok)),
            Ok(None) => None,
            Err(e) => Some(Err(e)),
        }
    }
}

/// `shlex.split(s, comments=False, posix=True)`.
pub fn split(s: &str, comments: bool, posix: bool) -> Result<Vec<String>, ShlexError> {
    let mut lex = Shlex::new(s, posix);
    lex.whitespace_split = true;
    if !comments {
        lex.commenters = Vec::new();
    }
    lex.tokens()
}

/// `shlex.quote(s)`.
pub fn quote(s: &str) -> String {
    if s.is_empty() {
        return "''".to_string();
    }
    let safe = s.chars().all(|c| {
        c.is_ascii_alphanumeric()
            || c == '_'
            || c == '@'
            || c == '%'
            || c == '+'
            || c == '='
            || c == ':'
            || c == ','
            || c == '.'
            || c == '/'
            || c == '-'
    });
    if safe {
        return s.to_string();
    }
    format!("'{}'", s.replace('\'', "'\"'\"'"))
}
