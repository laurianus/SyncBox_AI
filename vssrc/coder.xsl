<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" version="1.0">

  <xsl:template match="/document">
    <html>
      <head>
        <meta content="text/html; charset=utf-8" http-equiv="Content-Type"/>
        <title>
          <xsl:value-of select="title"/>
        </title>
      </head>
      <body bgcolor="#FFDE00">
        <h2>
          <xsl:value-of select="title"/>
        </h2>
        <xsl:value-of select="created/@name"/> 
        <xsl:value-of select="created/@time"/>
        <br/>
        <xsl:value-of select="signature/@generator"/> 
        <xsl:value-of select="signature/@version"/>
        <br/>Digital signature
        <xsl:value-of select="signature/@projectsignature"/>
        <h3>
          <xsl:value-of select="project/@title"/>
        </h3>
        <xsl:call-template name="PreserveLineBreak">
          <xsl:with-param name="text" select="project/@explanation"/>
        </xsl:call-template>
        <h4>
          <xsl:value-of select="project/options/@title"/>
        </h4>
        <xsl:for-each select="project/options/group">
          <h5>
            <xsl:value-of select="@name"/>
          </h5>
          <table>
            <xsl:for-each select="item">
              <tr>
                <td>
                  <xsl:value-of select="@name"/>
                </td>
                <td>
                  <xsl:value-of select="@value"/>
                </td>
              </tr>
            </xsl:for-each>
          </table>
        </xsl:for-each>
        <h4>
          <xsl:value-of select="project/model/@title"/>
        </h4>
        <xsl:for-each select="project/model/group">
          <h5>
            <xsl:value-of select="@name"/>
          </h5>
          <table>
            <xsl:for-each select="item">
              <tr>
                <td>
                  <xsl:value-of select="@name"/>
                </td>
                <td>
                  <xsl:value-of select="@value"/>
                </td>
              </tr>
            </xsl:for-each>
          </table>
        </xsl:for-each>
        <h4>
          <xsl:value-of select="project/statistics/@title"/>
        </h4>
        <xsl:for-each select="project/statistics/group">
          <h5>
            <xsl:value-of select="@name"/>
          </h5>
          <table>
            <xsl:for-each select="item">
              <tr>
                <td>
                  <xsl:value-of select="@name"/>
                </td>
                <td>
                  <xsl:value-of select="@value"/>
                </td>
              </tr>
            </xsl:for-each>
          </table>
        </xsl:for-each>
        <xsl:for-each select="project/system">
          <h3>
            <xsl:value-of select="@title"/>
          </h3>
          <xsl:call-template name="PreserveLineBreak">
            <xsl:with-param name="text" select="@explanation"/>
          </xsl:call-template>
          <h4>
            <xsl:value-of select="options/@title"/>
          </h4>
          <xsl:for-each select="options/group">
            <h5>
              <xsl:value-of select="@name"/>
            </h5>
            <table>
              <xsl:for-each select="item">
                <tr>
                  <td>
                    <xsl:value-of select="@name"/>
                  </td>
                  <td>
                    <xsl:value-of select="@value"/>
                  </td>
                </tr>
              </xsl:for-each>
            </table>
          </xsl:for-each>
          <h4>
            <xsl:value-of select="model/@title"/>
          </h4>
          <xsl:for-each select="model/group">
            <h5>
              <xsl:value-of select="@name"/>
            </h5>
            <table>
              <xsl:for-each select="item">
                <tr>
                  <td>
                    <xsl:value-of select="@name"/>
                  </td>
                  <td>
                    <xsl:value-of select="@value"/>
                  </td>
                </tr>
              </xsl:for-each>
            </table>
          </xsl:for-each>
          <h4>
            <xsl:value-of select="statistics/@title"/>
          </h4>
          <xsl:for-each select="statistics/group">
            <h5>
              <xsl:value-of select="@name"/>
            </h5>
            <table>
              <xsl:for-each select="item">
                <tr>
                  <td>
                    <xsl:value-of select="@name"/>
                  </td>
                  <td>
                    <xsl:value-of select="@value"/>
                  </td>
                </tr>
              </xsl:for-each>
            </table>
          </xsl:for-each>
        </xsl:for-each>
        <h3>
          <xsl:value-of select="summary/@title"/>
        </h3>
        <xsl:for-each select="summary/group">
          <h5>
            <xsl:value-of select="@name"/>
          </h5>
          <table>
            <xsl:for-each select="item">
              <tr>
                <td>
                  <xsl:value-of select="@name"/>
                </td>
                <td>
                  <xsl:value-of select="@value"/>
                </td>
              </tr>
            </xsl:for-each>
          </table>
        </xsl:for-each>
      </body>
    </html>
  </xsl:template>

  <xsl:template name="PreserveLineBreak">
    <xsl:param name="text"/>
    <xsl:choose>
      <xsl:when test="contains($text,'&#xA;')">
        <xsl:value-of select="substring-before($text,'&#xA;')"/>
        <br/>
        <xsl:call-template name="PreserveLineBreak">
          <xsl:with-param name="text" select="substring-after($text,'&#xA;')"/>
        </xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
        <xsl:value-of select="$text"/>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

</xsl:stylesheet>
