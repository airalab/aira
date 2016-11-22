/* eslint react/jsx-boolean-value: 0 */
import React, { Component } from 'react'
import { connect } from 'react-redux'
import _ from 'lodash'
import Autosuggest from 'react-autosuggest'

import styles from './style.css'

function escapeRegexCharacters(str) {
  return str.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}
function getSuggestionValue(suggestion) {
  return suggestion.value;
}
function renderSuggestion(suggestion) {
  return (
    <span>
      {suggestion.title}<br />
      <small>{suggestion.value}</small>
    </span>
  );
}

class Container extends Component {
  constructor() {
    super();

    this.state = {
      value: '',
      suggestions: []
    };
  }

  onSuggestionsFetchRequested = ({ value }) => {
    this.getSuggestions(value);
  };

  onSuggestionsClearRequested = () => {
    this.setState({
      suggestions: []
    });
  };

  getSuggestions(value) {
    const escapedValue = escapeRegexCharacters(value.trim());
    let suggestions = this.props.items;
    if (escapedValue !== '') {
      const regex = new RegExp('' + escapedValue, 'i');
      suggestions = _.filter(
        this.props.items,
        item => (regex.test(item.value) || regex.test(item.title))
      )
    }
    this.setState({
      suggestions
    });
  }

  render() {
    const { field, placeholder, onChange } = this.props;
    const { value, suggestions } = this.state;
    const inputProps = {
      placeholder,
      value,
      ...field,
      onChange: (evt, { newValue }) => {
        onChange(newValue);
        field.onChange(newValue);
      },
    };
    const theme = {
      container: styles['react-autosuggest__container'],
      containerOpen: styles['react-autosuggest__container--open'],
      input: 'form-control',
      suggestionsContainer: styles['react-autosuggest__suggestions-container'],
      suggestionsList: styles['react-autosuggest__suggestions-list'],
      suggestion: styles['react-autosuggest__suggestion'],
      suggestionFocused: styles['react-autosuggest__suggestion--focused']
    }

    return (<div className={styles['react-autosuggest__container']}>
      <Autosuggest
        suggestions={suggestions}
        alwaysRenderSuggestions={true}
        onSuggestionsFetchRequested={this.onSuggestionsFetchRequested}
        onSuggestionsClearRequested={this.onSuggestionsClearRequested}
        getSuggestionValue={getSuggestionValue}
        renderSuggestion={renderSuggestion}
        inputProps={inputProps}
        theme={theme}
      />
    </div>)
  }
}

function mapStateToProps(state, props) {
  return {
    field: props.field,
    placeholder: props.placeholder,
    onChange: props.onChange,
    items: props.items
  }
}

export default connect(mapStateToProps)(Container)
